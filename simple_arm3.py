"""Shared MAVLink helpers for freefall detection and force-arm recovery."""

from __future__ import annotations

import argparse
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


FORCE_ARM_MAGIC = 21196


class _FallbackMavlink:
    """Constants used when pymavlink is not installed during static checks."""

    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_DO_SET_MODE = 176
    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_RESULT_ACCEPTED = 0


MAVLINK = mavutil.mavlink if mavutil is not None else _FallbackMavlink()

# PX4 custom-mode values used with MAV_CMD_DO_SET_MODE.
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3

# MAVLink message IDs requested from PX4 during the test.
MAV_TYPE_GCS = getattr(MAVLINK, "MAV_TYPE_GCS", 6)
MAV_AUTOPILOT_INVALID = getattr(MAVLINK, "MAV_AUTOPILOT_INVALID", 8)
MSG_HEARTBEAT = getattr(MAVLINK, "MAVLINK_MSG_ID_HEARTBEAT", 0)
MSG_LOCAL_POSITION_NED = getattr(MAVLINK, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
MSG_GLOBAL_POSITION_INT = getattr(MAVLINK, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
MSG_ATTITUDE = getattr(MAVLINK, "MAVLINK_MSG_ID_ATTITUDE", 30)


@dataclass
class Config:
    """Runtime settings shared by the arm and hover scripts."""

    connection: str = "udpin:0.0.0.0:14560"
    connection_timeout_s: float = 8.0
    freefall_speed_mps: float = 2.0
    freefall_time_s: float = 0.15
    local_position_timeout_s: float = 0.5
    heartbeat_timeout_s: float = 2.0
    status_interval_s: float = 0.5
    history_window_s: float = 2.0


@dataclass
class SpeedSample:
    """One vertical-speed sample used to confirm sustained freefall."""

    time_s: float
    downward_speed_mps: float


@dataclass
class VehicleState:
    """Latest vehicle telemetry and command acknowledgements."""

    armed: bool = False
    mode: str = "UNKNOWN"
    heartbeat_time_s: Optional[float] = None
    altitude_m: Optional[float] = None
    altitude_time_s: Optional[float] = None
    downward_speed_mps: Optional[float] = None
    velocity_time_s: Optional[float] = None
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None
    rollspeed_rad_s: Optional[float] = None
    pitchspeed_rad_s: Optional[float] = None
    yawspeed_rad_s: Optional[float] = None
    attitude_time_s: Optional[float] = None
    last_status_text: Optional[str] = None
    status_text_time_s: Optional[float] = None
    command_acks: Dict[int, int] = field(default_factory=dict)
    command_ack_times_s: Dict[int, float] = field(default_factory=dict)
    message_counts: Dict[str, int] = field(default_factory=dict)
    message_first_time_s: Dict[str, float] = field(default_factory=dict)
    message_last_time_s: Dict[str, float] = field(default_factory=dict)
    speed_history: Deque[SpeedSample] = field(default_factory=deque)


PX4_MAIN_MODES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
}

# AUTO submodes are encoded inside PX4's custom heartbeat mode field.
PX4_AUTO_SUBMODES = {
    2: "AUTO.TAKEOFF",
    3: "AUTO.LOITER",
    4: "AUTO.MISSION",
    5: "AUTO.RTL",
    6: "AUTO.LAND",
}


def require_mavlink():
    """Return pymavlink or raise a clear install error."""

    if mavutil is None:
        raise RuntimeError("Install pymavlink before running this script.")
    return mavutil


def open_connection(connection: str, timeout_s: float = Config.connection_timeout_s):
    """Open MAVLink and wait for the PX4 vehicle heartbeat."""

    mav = require_mavlink()
    print(f"connecting: {connection}")
    master = mav.mavlink_connection(connection)
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        heartbeat = master.wait_heartbeat(timeout=max(0.0, deadline_s - time.monotonic()))
        if heartbeat is None:
            break
        if not is_gcs_heartbeat(heartbeat):
            master._initial_heartbeat = heartbeat
            print(f"connection established: {connection}")
            return master
    raise TimeoutError(f"No autopilot heartbeat on {connection}. Check MAVProxy --out and UDP port use.")


def is_gcs_heartbeat(message) -> bool:
    """Ignore ground-station heartbeats so we lock onto the vehicle."""

    return (
        int(getattr(message, "type", 0)) == MAV_TYPE_GCS
        or int(getattr(message, "autopilot", 0)) == MAV_AUTOPILOT_INVALID
    )


def request_message_rate(master, message_id: int, rate_hz: float) -> None:
    """Ask PX4 to stream one MAVLink message type at the requested rate."""

    interval_us = -1 if rate_hz <= 0 else int(1_000_000 / rate_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAVLINK.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0,
        0,
        0,
        0,
        0,
    )


def request_basic_telemetry(master) -> None:
    """Request the telemetry needed for vertical recovery and logs."""

    for message_id, rate_hz in (
        (MSG_HEARTBEAT, 2),
        (MSG_LOCAL_POSITION_NED, 50),
        (MSG_GLOBAL_POSITION_INT, 10),
        (MSG_ATTITUDE, 50),
    ):
        request_message_rate(master, message_id, rate_hz)
    print("[setup] requested telemetry: local_position=50Hz attitude=50Hz global_position=10Hz heartbeat=2Hz")


def update_state(master, state: VehicleState, config: Config, timeout_s: float = 0.0) -> None:
    """Read all waiting MAVLink messages and update VehicleState."""

    initial = getattr(master, "_initial_heartbeat", None)
    if initial is not None:
        master._initial_heartbeat = None
        process_message(master, state, config, initial, time.monotonic())

    message = master.recv_match(blocking=timeout_s > 0, timeout=timeout_s)
    if message is not None:
        process_message(master, state, config, message, time.monotonic())

    while True:
        message = master.recv_match(blocking=False, timeout=0)
        if message is None:
            return
        process_message(master, state, config, message, time.monotonic())


def process_message(master, state: VehicleState, config: Config, message, now_s: float) -> None:
    """Parse one MAVLink message into the compact VehicleState object."""

    message_type = message.get_type()
    if message_type == "BAD_DATA":
        return
    state.message_counts[message_type] = state.message_counts.get(message_type, 0) + 1
    state.message_first_time_s.setdefault(message_type, now_s)
    state.message_last_time_s[message_type] = now_s

    if message_type == "HEARTBEAT":
        if is_gcs_heartbeat(message):
            return
        base_mode = int(getattr(message, "base_mode", 0))
        state.heartbeat_time_s = now_s
        state.armed = bool(base_mode & MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED)
        state.mode = decode_px4_mode(message, str(getattr(master, "flightmode", state.mode)))
    elif message_type == "LOCAL_POSITION_NED":
        state.altitude_m = -float(getattr(message, "z", 0.0))
        state.altitude_time_s = now_s
        state.downward_speed_mps = float(getattr(message, "vz", 0.0))
        state.velocity_time_s = now_s
        add_speed_sample(state, config, now_s, state.downward_speed_mps)
    elif message_type == "GLOBAL_POSITION_INT" and not velocity_fresh(state, config, now_s):
        state.altitude_m = float(getattr(message, "relative_alt", 0.0)) / 1000.0
        state.altitude_time_s = now_s
        state.downward_speed_mps = float(getattr(message, "vz", 0.0)) / 100.0
        state.velocity_time_s = now_s
        add_speed_sample(state, config, now_s, state.downward_speed_mps)
    elif message_type == "ATTITUDE":
        state.roll_rad = float(getattr(message, "roll", 0.0))
        state.pitch_rad = float(getattr(message, "pitch", 0.0))
        state.yaw_rad = float(getattr(message, "yaw", 0.0))
        state.rollspeed_rad_s = float(getattr(message, "rollspeed", 0.0))
        state.pitchspeed_rad_s = float(getattr(message, "pitchspeed", 0.0))
        state.yawspeed_rad_s = float(getattr(message, "yawspeed", 0.0))
        state.attitude_time_s = now_s
    elif message_type == "STATUSTEXT":
        text = decode_status_text(message)
        if text:
            state.last_status_text = text
            state.status_text_time_s = now_s
            print(f"[px4] {text}")
    elif message_type == "COMMAND_ACK":
        command_id = int(getattr(message, "command", -1))
        state.command_acks[command_id] = int(getattr(message, "result", -1))
        state.command_ack_times_s[command_id] = now_s


def decode_px4_mode(message, fallback: str) -> str:
    """Decode PX4 mode values from HEARTBEAT.custom_mode."""

    custom_mode = int(getattr(message, "custom_mode", 0))
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    if main_mode == 4 and sub_mode in PX4_AUTO_SUBMODES:
        return PX4_AUTO_SUBMODES[sub_mode]
    return PX4_MAIN_MODES.get(main_mode, fallback)


def decode_status_text(message) -> str:
    """Convert PX4 STATUSTEXT payloads into normal Python strings."""

    text = getattr(message, "text", "")
    if isinstance(text, bytes):
        text = text.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
    return str(text).strip()


def add_speed_sample(state: VehicleState, config: Config, now_s: float, speed_mps: float) -> None:
    """Store recent vertical-speed samples for drop detection and acceleration logs."""

    state.speed_history.append(SpeedSample(now_s, speed_mps))
    while state.speed_history and state.speed_history[0].time_s < now_s - config.history_window_s:
        state.speed_history.popleft()


def velocity_fresh(state: VehicleState, config: Config, now_s: float) -> bool:
    """Return True when the vertical-speed sample is recent enough to trust."""

    return state.velocity_time_s is not None and now_s - state.velocity_time_s <= config.local_position_timeout_s


def altitude_fresh(state: VehicleState, config: Config, now_s: float) -> bool:
    """Return True when the altitude sample is recent enough to trust."""

    return state.altitude_time_s is not None and now_s - state.altitude_time_s <= config.local_position_timeout_s


def attitude_fresh(state: VehicleState, config: Config, now_s: float) -> bool:
    """Return True when the attitude sample is recent enough to trust."""

    return state.attitude_time_s is not None and now_s - state.attitude_time_s <= config.local_position_timeout_s


def freefall_detected(state: VehicleState, config: Config, now_s: float) -> bool:
    """Detect sustained downward velocity above the configured threshold."""

    if not velocity_fresh(state, config, now_s) or state.downward_speed_mps is None:
        return False
    if state.downward_speed_mps < config.freefall_speed_mps:
        return False
    cutoff_s = now_s - config.freefall_time_s
    samples = [sample for sample in state.speed_history if sample.time_s >= cutoff_s]
    return bool(samples) and all(sample.downward_speed_mps >= config.freefall_speed_mps for sample in samples)


def vertical_accel_mps2(state: VehicleState) -> Optional[float]:
    """Estimate vertical acceleration from the two newest speed samples."""

    if len(state.speed_history) < 2:
        return None
    older, newer = state.speed_history[-2], state.speed_history[-1]
    dt_s = newer.time_s - older.time_s
    if dt_s <= 0 or dt_s > 0.5:
        return None
    return (newer.downward_speed_mps - older.downward_speed_mps) / dt_s


def tilt_deg(state: VehicleState) -> Optional[float]:
    """Estimate thrust-axis tilt from world-up using roll and pitch."""

    if state.roll_rad is None or state.pitch_rad is None:
        return None
    alignment = math.cos(state.roll_rad) * math.cos(state.pitch_rad)
    return math.degrees(math.acos(max(-1.0, min(1.0, alignment))))


def max_body_rate_rad_s(state: VehicleState) -> Optional[float]:
    """Return the largest absolute roll/pitch/yaw rate if attitude data exists."""

    rates = (state.rollspeed_rad_s, state.pitchspeed_rad_s, state.yawspeed_rad_s)
    if any(rate is None for rate in rates):
        return None
    return max(abs(rate) for rate in rates if rate is not None)


def send_arm_command(master, force: bool = True) -> None:
    """Send an arm command, force-arming by default for drop recovery tests."""

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        FORCE_ARM_MAGIC if force else 0,
        0,
        0,
        0,
        0,
        0,
    )


def message_rate_hz(state: VehicleState, message_type: str, now_s: float) -> Optional[float]:
    """Estimate the observed rate of one MAVLink stream for logs."""

    first_s = state.message_first_time_s.get(message_type)
    count = state.message_counts.get(message_type, 0)
    if first_s is None or count < 2:
        return None
    return count / max(0.001, now_s - first_s)


def message_age_s(state: VehicleState, message_type: str, now_s: float) -> Optional[float]:
    """Return the age of the newest message of this type."""

    last_s = state.message_last_time_s.get(message_type)
    return None if last_s is None else now_s - last_s


def recent_status_text(state: VehicleState, now_s: float, max_age_s: float = 5.0) -> str:
    """Return recent PX4 status text, or 'none' if it is stale."""

    if state.status_text_time_s is None or state.last_status_text is None:
        return "none"
    return state.last_status_text if now_s - state.status_text_time_s <= max_age_s else "none"


def fmt(value: Optional[float], digits: int = 1) -> str:
    """Format optional telemetry values for compact console output."""

    return "n/a" if value is None else f"{value:.{digits}f}"


def status_line(state: VehicleState, config: Config, now_s: float) -> str:
    """Build the common one-line flight-state summary."""

    lpos_hz = message_rate_hz(state, "LOCAL_POSITION_NED", now_s)
    lpos_age = message_age_s(state, "LOCAL_POSITION_NED", now_s)
    attitude_hz = message_rate_hz(state, "ATTITUDE", now_s)
    attitude_age = message_age_s(state, "ATTITUDE", now_s)
    hb_age = None if state.heartbeat_time_s is None else now_s - state.heartbeat_time_s
    hb_state = "ok" if hb_age is not None and hb_age <= config.heartbeat_timeout_s else "stale"
    return (
        f"alt={fmt(state.altitude_m)}m "
        f"vz={fmt(state.downward_speed_mps)}m/s "
        f"mode={state.mode} "
        f"armed={state.armed} "
        f"local={'ok' if altitude_fresh(state, config, now_s) else 'stale'} "
        f"lpos={fmt(lpos_hz, 0)}Hz/{fmt(lpos_age, 2)}s "
        f"att={fmt(attitude_hz, 0)}Hz/{fmt(attitude_age, 2)}s "
        f"tilt={fmt(tilt_deg(state), 0)}deg "
        f"vel_age={fmt(None if state.velocity_time_s is None else now_s - state.velocity_time_s, 2)}s "
        f"hb={fmt(hb_age)}s/{hb_state}"
    )


def run(config: Config) -> int:
    """Arm-only test: wait for freefall, then send force-arm."""

    master = open_connection(config.connection, config.connection_timeout_s)
    request_basic_telemetry(master)
    state = VehicleState()
    last_status_s = 0.0
    print(f"[setup] freefall trigger: vz>{config.freefall_speed_mps:.1f}m/s for {config.freefall_time_s:.2f}s")
    while True:
        update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()
        if now_s - last_status_s >= config.status_interval_s:
            print(f"[wait] {status_line(state, config, now_s)} px4='{recent_status_text(state, now_s)}'")
            last_status_s = now_s
        if freefall_detected(state, config, now_s):
            print(f"[drop] {status_line(state, config, now_s)}")
            send_arm_command(master, force=True)
            return 0


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the CLI for the arm-only helper."""

    parser = argparse.ArgumentParser(description="Minimal PX4 freefall force-arm helper.")
    parser.add_argument("--connection", default=Config.connection)
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps)
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s)
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s)
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    """Convert parsed CLI arguments into Config."""

    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
    )


def cli() -> int:
    """Command-line entry point."""

    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

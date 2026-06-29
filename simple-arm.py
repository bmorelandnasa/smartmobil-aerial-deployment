from __future__ import annotations

import argparse
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


FORCE_ARM_MAGIC = 21196


class _FallbackMavlink:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_RESULT_ACCEPTED = 0
    MAV_RESULT_IN_PROGRESS = 5
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_CMD_DO_SET_MODE = 176
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE = 1 << 7


MAVLINK = mavutil.mavlink if mavutil is not None else _FallbackMavlink()


@dataclass
class Config:
    connection: str = "udpin:127.0.0.1:14560"
    freefall_speed_mps: float = 5.0
    freefall_time_s: float = 0.35
    telemetry_timeout_s: float = 0.5
    status_interval_s: float = 0.5
    ack_timeout_s: float = 2.0
    history_window_s: float = 2.0


@dataclass
class SpeedSample:
    time_s: float
    downward_speed_mps: float


@dataclass
class VehicleState:
    armed: bool = False
    mode: str = "UNKNOWN"
    heartbeat_time_s: Optional[float] = None
    downward_speed_mps: Optional[float] = None
    ground_speed_mps: Optional[float] = None
    velocity_source: Optional[str] = None
    velocity_time_s: Optional[float] = None
    speed_history: Deque[SpeedSample] = field(default_factory=deque)


def require_mavlink():
    if mavutil is None:
        raise RuntimeError("pymavlink is not installed. Install it before running this script.")
    return mavutil


def format_number(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.2f}"


def open_connection(connection_string: str):
    mav = require_mavlink()
    print(f"connecting: {connection_string}")
    master = mav.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(f"connection established: {connection_string}")
    return master


def update_state(master, state: VehicleState, config: Config, timeout_s: float = 0.0) -> None:
    message = master.recv_match(blocking=timeout_s > 0.0, timeout=timeout_s)
    if message is not None:
        process_message(master, state, config, message, time.monotonic())

    while True:
        message = master.recv_match(blocking=False, timeout=0)
        if message is None:
            return
        process_message(master, state, config, message, time.monotonic())


def process_message(master, state: VehicleState, config: Config, message, now_s: float) -> None:
    message_type = message.get_type()
    if message_type == "BAD_DATA":
        return

    if message_type == "HEARTBEAT":
        state.heartbeat_time_s = now_s
        state.armed = bool(message.base_mode & MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED)
        state.mode = str(getattr(master, "flightmode", state.mode))
        return

    if message_type == "LOCAL_POSITION_NED":
        downward_speed_mps = float(getattr(message, "vz", 0.0))
        vx = float(getattr(message, "vx", 0.0))
        vy = float(getattr(message, "vy", 0.0))
        state.downward_speed_mps = downward_speed_mps
        state.ground_speed_mps = (vx * vx + vy * vy) ** 0.5
        state.velocity_source = "LOCAL_POSITION_NED"
        state.velocity_time_s = now_s
        append_speed_sample(state, config, now_s, downward_speed_mps)
        return

    if message_type == "GLOBAL_POSITION_INT":
        downward_speed_mps = float(getattr(message, "vz", 0.0)) / 100.0
        state.downward_speed_mps = downward_speed_mps
        state.ground_speed_mps = float(getattr(message, "vel", 0.0)) / 100.0
        state.velocity_source = "GLOBAL_POSITION_INT"
        state.velocity_time_s = now_s
        append_speed_sample(state, config, now_s, downward_speed_mps)


def append_speed_sample(state: VehicleState, config: Config, now_s: float, downward_speed_mps: float) -> None:
    state.speed_history.append(SpeedSample(time_s=now_s, downward_speed_mps=downward_speed_mps))
    cutoff_s = now_s - config.history_window_s
    while state.speed_history and state.speed_history[0].time_s < cutoff_s:
        state.speed_history.popleft()


def velocity_is_fresh(state: VehicleState, config: Config, now_s: float) -> bool:
    return state.velocity_time_s is not None and now_s - state.velocity_time_s <= config.telemetry_timeout_s


def freefall_detected(state: VehicleState, config: Config, now_s: float) -> bool:
    if not velocity_is_fresh(state, config, now_s):
        return False
    if state.downward_speed_mps is None or state.downward_speed_mps < config.freefall_speed_mps:
        return False

    window_start_s = now_s - config.freefall_time_s
    relevant = [sample for sample in state.speed_history if sample.time_s <= now_s]
    if not relevant:
        return False

    anchor = [sample for sample in relevant if sample.time_s <= window_start_s]
    trailing = [sample for sample in relevant if sample.time_s >= window_start_s]
    if not anchor or not trailing:
        return False

    for sample in [anchor[-1], *trailing]:
        if sample.downward_speed_mps < config.freefall_speed_mps:
            return False
    return True


def status_line(state: VehicleState, config: Config, now_s: float) -> str:
    heartbeat_fresh = state.heartbeat_time_s is not None and now_s - state.heartbeat_time_s <= config.telemetry_timeout_s
    return (
        "status: "
        f"heartbeat={'fresh' if heartbeat_fresh else 'stale'} "
        f"armed={state.armed} "
        f"mode={state.mode} "
        f"source={state.velocity_source or 'none'} "
        f"down_mps={format_number(state.downward_speed_mps)} "
        f"ground_mps={format_number(state.ground_speed_mps)}"
    )


def wait_for_ack(master, state: VehicleState, config: Config, command_id: int) -> bool:
    deadline_s = time.monotonic() + config.ack_timeout_s
    while time.monotonic() < deadline_s:
        message = master.recv_match(blocking=True, timeout=min(0.1, deadline_s - time.monotonic()))
        if message is None:
            continue
        process_message(master, state, config, message, time.monotonic())
        if message.get_type() != "COMMAND_ACK":
            continue
        if getattr(message, "command", None) != command_id:
            continue
        result = getattr(message, "result", None)
        print(f"command ack: command={command_id} result={result}")
        return result in (
            MAVLINK.MAV_RESULT_ACCEPTED,
            MAVLINK.MAV_RESULT_IN_PROGRESS,
        )
    print(f"command ack timeout: command={command_id}")
    return False


def send_arm_command(master, force: bool) -> None:
    force_value = FORCE_ARM_MAGIC if force else 0
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        force_value,
        0,
        0,
        0,
        0,
        0,
    )


def force_arm(master, state: VehicleState, config: Config) -> bool:
    print("arm attempt: normal")
    send_arm_command(master, force=False)
    if wait_for_ack(master, state, config, MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM) or state.armed:
        print("arm accepted")
        return True

    print("arm denied")
    print("arm attempt: force")
    send_arm_command(master, force=True)
    if wait_for_ack(master, state, config, MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM) or state.armed:
        print("force-arm accepted")
        return True

    print("force-arm denied")
    return False


def run(config: Config) -> int:
    master = open_connection(config.connection)
    state = VehicleState()
    last_status_s = 0.0

    print(
        "freefall trigger: "
        f"threshold={config.freefall_speed_mps:.2f}m/s "
        f"duration={config.freefall_time_s:.2f}s"
    )

    while True:
        update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print(status_line(state, config, now_s))
            last_status_s = now_s

        if freefall_detected(state, config, now_s):
            print("trigger fired: freefall")
            return 0 if force_arm(master, state, config) else 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 freefall force-arm script.")
    parser.add_argument("--connection", default=Config.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s, help="Status print interval in seconds.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
    )


def cli() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run(config_from_args(args))


if __name__ == "__main__":
    raise SystemExit(cli())

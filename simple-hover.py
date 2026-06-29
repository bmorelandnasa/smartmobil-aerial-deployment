from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import simple_px4_freefall_arm as common


@dataclass
class HoverConfig(common.Config):
    stream_rate_hz: float = 20.0
    prime_stream_s: float = 1.0
    recovery_duration_s: float = 6.0
    recovery_thrust: float = 0.72
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    offboard_retry_interval_s: float = 0.5
    offboard_timeout_s: float = 3.0


def send_hover_setpoint(master, config: HoverConfig) -> None:
    master.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        common.MAVLINK.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
        [1.0, 0.0, 0.0, 0.0],
        config.roll_rate_rad_s,
        config.pitch_rate_rad_s,
        config.yaw_rate_rad_s,
        config.recovery_thrust,
    )


def request_offboard_mode(master) -> None:
    mapping = {}
    if hasattr(master, "mode_mapping"):
        try:
            mapping = master.mode_mapping() or {}
        except TypeError:
            mapping = {}

    offboard_mode = mapping.get("OFFBOARD") if mapping else None
    if offboard_mode is not None and hasattr(master, "set_mode"):
        master.set_mode(offboard_mode)
        return

    if hasattr(master.mav, "set_mode_send"):
        master.mav.set_mode_send(
            master.target_system,
            common.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            common.MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        common.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        common.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        common.MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0,
        0,
        0,
        0,
        0,
    )


def stream_for_time(master, state: common.VehicleState, config: HoverConfig, duration_s: float, request_offboard: bool) -> None:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + duration_s
    last_status_s = 0.0
    last_offboard_request_s = 0.0

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        send_hover_setpoint(master, config)
        common.update_state(master, state, config, timeout_s=0.0)

        if request_offboard and now_s - last_offboard_request_s >= config.offboard_retry_interval_s:
            request_offboard_mode(master)
            last_offboard_request_s = now_s

        if now_s - last_status_s >= config.status_interval_s:
            print(common.status_line(state, config, now_s))
            last_status_s = now_s

        time.sleep(interval_s)


def offboard_active(state: common.VehicleState) -> bool:
    return state.mode.upper() == "OFFBOARD"


def enter_offboard(master, state: common.VehicleState, config: HoverConfig) -> bool:
    deadline_s = time.monotonic() + config.offboard_timeout_s
    while time.monotonic() < deadline_s:
        request_offboard_mode(master)
        stream_for_time(master, state, config, config.offboard_retry_interval_s, request_offboard=False)
        if offboard_active(state):
            print("offboard accepted")
            return True
    print("offboard denied")
    return False


def run(config: HoverConfig) -> int:
    master = common.open_connection(config.connection)
    state = common.VehicleState()
    last_status_s = 0.0

    print(
        "freefall trigger: "
        f"threshold={config.freefall_speed_mps:.2f}m/s "
        f"duration={config.freefall_time_s:.2f}s"
    )
    print(
        "hover recovery: "
        f"thrust={config.recovery_thrust:.2f} "
        f"stream_rate={config.stream_rate_hz:.1f}Hz "
        f"duration={config.recovery_duration_s:.1f}s"
    )

    while True:
        common.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print(common.status_line(state, config, now_s))
            last_status_s = now_s

        if not common.freefall_detected(state, config, now_s):
            continue

        print("trigger fired: freefall")
        print("streaming setpoints before arming")
        stream_for_time(master, state, config, config.prime_stream_s, request_offboard=False)

        if not common.force_arm(master, state, config):
            return 1

        if not enter_offboard(master, state, config):
            return 1

        print("recovery streaming active")
        stream_for_time(master, state, config, config.recovery_duration_s, request_offboard=True)
        print("recovery streaming stopped")
        return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 freefall hover-recovery script.")
    parser.add_argument("--connection", default=HoverConfig.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=HoverConfig.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=HoverConfig.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=HoverConfig.status_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--thrust", type=float, default=HoverConfig.recovery_thrust, help="Normalized thrust to hold after the trigger.")
    parser.add_argument("--stream-rate", type=float, default=HoverConfig.stream_rate_hz, help="Offboard setpoint stream rate in Hz.")
    parser.add_argument("--prime-stream", type=float, default=HoverConfig.prime_stream_s, help="How long to stream before arm/offboard commands.")
    parser.add_argument("--recovery-duration", type=float, default=HoverConfig.recovery_duration_s, help="How long to keep hover recovery active.")
    return parser


def config_from_args(args: argparse.Namespace) -> HoverConfig:
    return HoverConfig(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
        recovery_thrust=args.thrust,
        stream_rate_hz=args.stream_rate,
        prime_stream_s=args.prime_stream,
        recovery_duration_s=args.recovery_duration,
    )


def cli() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run(config_from_args(args))


if __name__ == "__main__":
    raise SystemExit(cli())

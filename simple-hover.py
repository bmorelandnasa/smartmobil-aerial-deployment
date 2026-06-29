from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import simple_arm as arm


@dataclass
class Config(arm.Config):
    stream_rate_hz: float = 20.0
    prime_stream_s: float = 1.0
    recovery_duration_s: float = 8.0
    hover_thrust: float = 0.72
    catch_thrust: float = 0.98
    min_thrust: float = 0.65
    max_thrust: float = 1.0
    descent_gain: float = 0.06
    catch_duration_s: float = 1.5
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    offboard_retry_interval_s: float = 0.5
    offboard_timeout_s: float = 3.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def commanded_thrust(state: arm.VehicleState, config: Config, recovery_started_s: float, now_s: float) -> float:
    downward_speed = max(0.0, state.downward_speed_mps or 0.0)

    if now_s - recovery_started_s < config.catch_duration_s and downward_speed > 0.5:
        return config.catch_thrust

    thrust = config.hover_thrust + downward_speed * config.descent_gain
    return clamp(thrust, config.min_thrust, config.max_thrust)


def send_hover_setpoint(master, config: Config, thrust: float) -> None:
    master.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        arm.MAVLINK.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
        [1.0, 0.0, 0.0, 0.0],
        config.roll_rate_rad_s,
        config.pitch_rate_rad_s,
        config.yaw_rate_rad_s,
        thrust,
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
            arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            arm.MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        arm.MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0,
        0,
        0,
        0,
        0,
    )


def stream_for_time(master, state: arm.VehicleState, config: Config, duration_s: float, recovery_started_s: float, request_offboard: bool) -> None:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + duration_s
    last_status_s = 0.0
    last_offboard_request_s = 0.0

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        thrust = commanded_thrust(state, config, recovery_started_s, now_s)
        send_hover_setpoint(master, config, thrust)
        arm.update_state(master, state, config, timeout_s=0.0)

        if request_offboard and now_s - last_offboard_request_s >= config.offboard_retry_interval_s:
            request_offboard_mode(master)
            last_offboard_request_s = now_s

        if now_s - last_status_s >= config.status_interval_s:
            print(f"{arm.status_line(state, config, now_s)} thrust_cmd={thrust:.2f}")
            last_status_s = now_s

        time.sleep(interval_s)


def offboard_active(state: arm.VehicleState) -> bool:
    return state.mode.upper() == "OFFBOARD"


def enter_offboard(master, state: arm.VehicleState, config: Config, recovery_started_s: float) -> bool:
    deadline_s = time.monotonic() + config.offboard_timeout_s
    while time.monotonic() < deadline_s:
        request_offboard_mode(master)
        stream_for_time(master, state, config, config.offboard_retry_interval_s, recovery_started_s, request_offboard=False)
        if offboard_active(state):
            print("offboard accepted")
            return True
    print("offboard denied")
    return False


def run(config: Config) -> int:
    master = arm.open_connection(config.connection)
    state = arm.VehicleState()
    last_status_s = 0.0

    print(f"freefall trigger: threshold={config.freefall_speed_mps:.2f}m/s duration={config.freefall_time_s:.2f}s")
    print(
        "hover recovery: "
        f"hover_thrust={config.hover_thrust:.2f} "
        f"catch_thrust={config.catch_thrust:.2f} "
        f"gain={config.descent_gain:.3f} "
        f"duration={config.recovery_duration_s:.1f}s"
    )

    while True:
        arm.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print(arm.status_line(state, config, now_s))
            last_status_s = now_s

        if not arm.freefall_detected(state, config, now_s):
            continue

        print("trigger fired: freefall")
        print("streaming setpoints before arming")
        recovery_started_s = time.monotonic()
        stream_for_time(master, state, config, config.prime_stream_s, recovery_started_s, request_offboard=False)

        if not arm.force_arm(master, state, config):
            return 1

        if not enter_offboard(master, state, config, recovery_started_s):
            return 1

        print("recovery streaming active")
        stream_for_time(master, state, config, config.recovery_duration_s, recovery_started_s, request_offboard=True)
        print("recovery streaming stopped")
        return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 freefall hover-recovery script.")
    parser.add_argument("--connection", default=Config.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--hover-thrust", type=float, default=Config.hover_thrust, help="Base thrust after the initial catch.")
    parser.add_argument("--catch-thrust", type=float, default=Config.catch_thrust, help="High thrust used right after rearming during a fast fall.")
    parser.add_argument("--descent-gain", type=float, default=Config.descent_gain, help="Extra thrust per m/s of downward speed.")
    parser.add_argument("--catch-duration", type=float, default=Config.catch_duration_s, help="How long to hold catch thrust after trigger.")
    parser.add_argument("--stream-rate", type=float, default=Config.stream_rate_hz, help="Offboard setpoint stream rate in Hz.")
    parser.add_argument("--prime-stream", type=float, default=Config.prime_stream_s, help="How long to stream before arm/offboard commands.")
    parser.add_argument("--recovery-duration", type=float, default=Config.recovery_duration_s, help="How long to keep recovery active.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
        hover_thrust=args.hover_thrust,
        catch_thrust=args.catch_thrust,
        descent_gain=args.descent_gain,
        catch_duration_s=args.catch_duration,
        stream_rate_hz=args.stream_rate,
        prime_stream_s=args.prime_stream,
        recovery_duration_s=args.recovery_duration,
    )


def cli() -> int:
    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

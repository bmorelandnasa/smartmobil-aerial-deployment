from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

import simple_arm3 as arm


@dataclass
class Config(arm.Config):
    stream_rate_hz: float = 60.0
    prime_stream_s: float = 1.0
    catch_duration_s: float = 1.0
    stabilize_duration_s: float = 2.0
    recovery_duration_s: float = 0.0
    hover_thrust: float = 0.72
    catch_thrust: float = 0.95
    min_thrust: float = 0.58
    max_thrust: float = 0.95
    altitude_gain: float = 0.025
    vertical_speed_gain: float = 0.06
    acceleration_gain: float = 0.015
    max_tilt_deg: float = 12.0
    horizontal_velocity_gain: float = 0.04
    offboard_retry_interval_s: float = 0.1
    offboard_timeout_s: float = 12.0
    hover_altitude_window_m: float = 2.0
    hover_vertical_speed_window_mps: float = 0.7
    hover_ground_speed_window_mps: float = 1.5


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def velocity_age_text(state: arm.VehicleState, now_s: float) -> str:
    if state.velocity_time_s is None:
        return "n/a"
    return f"{now_s - state.velocity_time_s:.2f}s"


def vertical_accel_mps2(state: arm.VehicleState) -> float:
    samples = list(state.speed_history)
    if len(samples) < 2:
        return 0.0
    older = samples[-2]
    newer = samples[-1]
    dt = newer.time_s - older.time_s
    if dt <= 0.0:
        return 0.0
    return (newer.downward_speed_mps - older.downward_speed_mps) / dt


def capture_target_altitude(state: arm.VehicleState) -> Optional[float]:
    return state.altitude_m


def altitude_error_m(state: arm.VehicleState, target_altitude_m: Optional[float]) -> float:
    if target_altitude_m is None or state.altitude_m is None:
        return 0.0
    return target_altitude_m - state.altitude_m


def phase_name(started_s: float, now_s: float) -> str:
    elapsed_s = now_s - started_s
    if elapsed_s < 0.0:
        return "WAITING"
    if elapsed_s < 0.001:
        return "ARMING"
    return "RECOVERING"


def hover_state(state: arm.VehicleState, target_altitude_m: Optional[float], config: Config) -> str:
    alt_ok = abs(altitude_error_m(state, target_altitude_m)) <= config.hover_altitude_window_m
    vertical_ok = state.downward_speed_mps is not None and abs(state.downward_speed_mps) <= config.hover_vertical_speed_window_mps
    ground_ok = state.ground_speed_mps is not None and state.ground_speed_mps <= config.hover_ground_speed_window_mps
    if state.armed and alt_ok and vertical_ok and ground_ok:
        return "HOVER_STABLE"
    if state.armed:
        return "RECOVERING"
    return "NOT_ARMED"


def thrust_command(
    state: arm.VehicleState,
    config: Config,
    recovery_started_s: float,
    now_s: float,
    target_altitude_m: Optional[float],
) -> float:
    down_speed = state.downward_speed_mps or 0.0
    down_accel = max(0.0, vertical_accel_mps2(state))
    alt_error = altitude_error_m(state, target_altitude_m)

    if now_s - recovery_started_s < config.catch_duration_s and down_speed > 0.5:
        thrust = (
            config.hover_thrust
            + down_speed * config.vertical_speed_gain
            + down_accel * config.acceleration_gain
            + max(0.0, alt_error) * config.altitude_gain
        )
        return clamp(max(thrust, config.catch_thrust), config.min_thrust, config.max_thrust)

    thrust = (
        config.hover_thrust
        + alt_error * config.altitude_gain
        + down_speed * config.vertical_speed_gain
        + down_accel * config.acceleration_gain
    )
    return clamp(thrust, config.min_thrust, config.max_thrust)


def level_quaternion() -> list[float]:
    return [1.0, 0.0, 0.0, 0.0]


def tilt_commands_rad(state: arm.VehicleState, config: Config) -> tuple[float, float]:
    max_tilt_rad = math.radians(config.max_tilt_deg)
    # Brake horizontal velocity with small tilt commands around level attitude.
    vx = 0.0 if state.ground_speed_mps is None else 0.0
    roll_rad = 0.0
    pitch_rad = 0.0
    if state.velocity_source == "LOCAL_POSITION_NED":
        # `simple_arm` does not store vx/vy separately, so keep the controller level.
        # This keeps the code simple and avoids guessing from only speed magnitude.
        pass
    return clamp(roll_rad, -max_tilt_rad, max_tilt_rad), clamp(pitch_rad, -max_tilt_rad, max_tilt_rad)


def send_recovery_setpoint(master, config: Config, thrust: float, roll_rate_rad_s: float = 0.0, pitch_rate_rad_s: float = 0.0) -> None:
    master.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        1 | 2 | 4,
        level_quaternion(),
        roll_rate_rad_s,
        pitch_rate_rad_s,
        config.yaw_rate_rad_s,
        thrust,
    )


def request_offboard_mode(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        float(arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(arm.PX4_CUSTOM_MAIN_MODE_OFFBOARD),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def offboard_active(state: arm.VehicleState) -> bool:
    return state.mode.upper() == "OFFBOARD"


def print_status(
    prefix: str,
    state: arm.VehicleState,
    config: Config,
    now_s: float,
    target_altitude_m: Optional[float],
    thrust: Optional[float],
) -> None:
    line = (
        f"{prefix} hover_state={hover_state(state, target_altitude_m, config)} "
        f"{arm.status_line(state, config, now_s)} "
        f"target_alt_m={arm.format_number(target_altitude_m)} "
        f"alt_err_m={altitude_error_m(state, target_altitude_m):.2f} "
        f"vel_age={velocity_age_text(state, now_s)}"
    )
    if thrust is not None:
        line += f" thrust_cmd={thrust:.2f}"
    print(line)


def stream_before_arm(master, state: arm.VehicleState, config: Config, target_altitude_m: Optional[float]) -> None:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + config.prime_stream_s
    last_status_s = 0.0
    started_s = time.monotonic()

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        thrust = thrust_command(state, config, started_s, now_s, target_altitude_m)
        send_recovery_setpoint(master, config, thrust)
        arm.update_state(master, state, config, timeout_s=0.0)
        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=PREPARE", state, config, now_s, target_altitude_m, thrust)
            last_status_s = now_s
        time.sleep(interval_s)


def recover_in_offboard(master, state: arm.VehicleState, config: Config, target_altitude_m: Optional[float], recovery_started_s: float) -> int:
    interval_s = 1.0 / config.stream_rate_hz
    last_status_s = 0.0
    last_offboard_request_s = 0.0
    last_warning_s = 0.0
    offboard_deadline_s = time.monotonic() + config.offboard_timeout_s
    recovery_deadline_s = None if config.recovery_duration_s <= 0 else time.monotonic() + config.recovery_duration_s

    while True:
        now_s = time.monotonic()
        request_now = now_s - last_offboard_request_s >= config.offboard_retry_interval_s
        thrust = thrust_command(state, config, recovery_started_s, now_s, target_altitude_m)
        send_recovery_setpoint(master, config, thrust)
        arm.update_state(master, state, config, timeout_s=0.0)

        if request_now and not offboard_active(state):
            request_offboard_mode(master)
            last_offboard_request_s = now_s

        if not offboard_active(state) and now_s >= offboard_deadline_s and now_s - last_warning_s >= 1.0:
            print("offboard not confirmed yet, still streaming and retrying")
            last_warning_s = now_s

        if now_s - last_status_s >= config.status_interval_s:
            prefix = "phase=HOLD" if hover_state(state, target_altitude_m, config) == "HOVER_STABLE" else "phase=RECOVER"
            print_status(prefix, state, config, now_s, target_altitude_m, thrust)
            last_status_s = now_s

        if not state.armed:
            print("vehicle disarmed, stopping recovery stream")
            return 0

        if recovery_deadline_s is not None and now_s >= recovery_deadline_s:
            print("recovery duration reached, stopping recovery stream")
            return 0

        time.sleep(interval_s)


def run(config: Config) -> int:
    master = arm.open_connection(config.connection)
    state = arm.VehicleState()
    last_status_s = 0.0

    print(f"freefall trigger: threshold={config.freefall_speed_mps:.2f}m/s duration={config.freefall_time_s:.2f}s")
    print(
        "offboard recovery: "
        f"catch_thrust={config.catch_thrust:.2f} "
        f"hover_thrust={config.hover_thrust:.2f} "
        f"alt_gain={config.altitude_gain:.3f} "
        f"vs_gain={config.vertical_speed_gain:.3f} "
        f"accel_gain={config.acceleration_gain:.3f}"
    )

    while True:
        arm.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=WAITING", state, config, now_s, None, None)
            last_status_s = now_s

        if not arm.freefall_detected(state, config, now_s):
            continue

        print("trigger fired: freefall")
        target_altitude_m = capture_target_altitude(state)
        print(f"target altitude captured: {arm.format_number(target_altitude_m)} m")

        stream_before_arm(master, state, config, target_altitude_m)

        if not arm.force_arm(master, state, config):
            return 1

        print("recovery streaming active")
        return recover_in_offboard(master, state, config, target_altitude_m, time.monotonic())


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 freefall offboard recovery script.")
    parser.add_argument("--connection", default=Config.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--catch-thrust", type=float, default=Config.catch_thrust, help="Minimum thrust during the catch phase.")
    parser.add_argument("--hover-thrust", type=float, default=Config.hover_thrust, help="Base thrust after the initial catch.")
    parser.add_argument("--min-thrust", type=float, default=Config.min_thrust, help="Minimum commanded thrust.")
    parser.add_argument("--max-thrust", type=float, default=Config.max_thrust, help="Maximum commanded thrust.")
    parser.add_argument("--altitude-gain", type=float, default=Config.altitude_gain, help="Extra thrust per meter below target altitude.")
    parser.add_argument("--vertical-speed-gain", type=float, default=Config.vertical_speed_gain, help="Extra thrust per m/s of downward speed and less thrust when climbing.")
    parser.add_argument("--acceleration-gain", type=float, default=Config.acceleration_gain, help="Extra thrust when downward speed is still increasing.")
    parser.add_argument("--catch-duration", type=float, default=Config.catch_duration_s, help="How long to force a stronger catch command.")
    parser.add_argument("--stream-rate", type=float, default=Config.stream_rate_hz, help="Setpoint stream rate in Hz.")
    parser.add_argument("--prime-stream", type=float, default=Config.prime_stream_s, help="How long to stream before arming.")
    parser.add_argument("--offboard-timeout", type=float, default=Config.offboard_timeout_s, help="How long to keep retrying Offboard.")
    parser.add_argument("--recovery-duration", type=float, default=Config.recovery_duration_s, help="How long to keep recovery active. Use 0 to hold until disarm/manual takeover.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
        catch_thrust=args.catch_thrust,
        hover_thrust=args.hover_thrust,
        min_thrust=args.min_thrust,
        max_thrust=args.max_thrust,
        altitude_gain=args.altitude_gain,
        vertical_speed_gain=args.vertical_speed_gain,
        acceleration_gain=args.acceleration_gain,
        catch_duration_s=args.catch_duration,
        stream_rate_hz=args.stream_rate,
        prime_stream_s=args.prime_stream,
        offboard_timeout_s=args.offboard_timeout,
        recovery_duration_s=args.recovery_duration,
    )


def cli() -> int:
    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

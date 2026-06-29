from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import simple_arm2 as arm


PX4_CUSTOM_MAIN_MODE_ALTCTL = 2


@dataclass
class Config(arm.Config):
    stream_rate_hz: float = 50.0
    prime_stream_s: float = 0.5
    catch_duration_s: float = 0.6
    catch_thrust: float = 0.95
    hover_thrust: float = 0.72
    min_thrust: float = 0.60
    max_thrust: float = 0.95
    vertical_speed_gain: float = 0.05
    acceleration_guard_gain: float = 0.02
    altctl_retry_interval_s: float = 0.2
    altctl_timeout_s: float = 8.0
    monitor_duration_s: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def velocity_age_text(state: arm.VehicleState, now_s: float) -> str:
    if state.velocity_time_s is None:
        return "n/a"
    return f"{now_s - state.velocity_time_s:.2f}s"


def infer_vertical_accel_mps2(state: arm.VehicleState) -> float:
    history = list(state.speed_history)
    if len(history) < 2:
        return 0.0
    older = history[-2]
    newer = history[-1]
    dt = newer.time_s - older.time_s
    if dt <= 0.0:
        return 0.0
    return (newer.downward_speed_mps - older.downward_speed_mps) / dt


def catch_thrust_command(state: arm.VehicleState, config: Config, started_s: float, now_s: float) -> float:
    downward_speed = max(0.0, state.downward_speed_mps or 0.0)
    downward_accel = max(0.0, infer_vertical_accel_mps2(state))

    if now_s - started_s < config.catch_duration_s and downward_speed > 0.5:
        thrust = (
            config.hover_thrust
            + downward_speed * config.vertical_speed_gain
            + downward_accel * config.acceleration_guard_gain
        )
        return clamp(max(thrust, config.catch_thrust), config.min_thrust, config.max_thrust)

    thrust = config.hover_thrust + downward_speed * config.vertical_speed_gain
    return clamp(thrust, config.min_thrust, config.max_thrust)


def send_catch_setpoint(master, config: Config, thrust: float) -> None:
    master.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        1 | 2 | 4,
        [1.0, 0.0, 0.0, 0.0],
        config.roll_rate_rad_s,
        config.pitch_rate_rad_s,
        config.yaw_rate_rad_s,
        thrust,
    )


def request_altctl_mode(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        float(arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(PX4_CUSTOM_MAIN_MODE_ALTCTL),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def altctl_active(state: arm.VehicleState) -> bool:
    mode = state.mode.upper()
    return mode in {"ALTCTL", "ALTITUDE", "ALTITUDE CONTROL"}


def print_status(prefix: str, state: arm.VehicleState, config: Config, now_s: float, thrust: float | None) -> None:
    line = f"{prefix} {arm.status_line(state, config, now_s)} vel_age={velocity_age_text(state, now_s)}"
    if thrust is not None:
        line += f" thrust_cmd={thrust:.2f}"
    print(line)


def prime_and_catch(master, state: arm.VehicleState, config: Config, started_s: float) -> None:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + config.prime_stream_s
    last_status_s = 0.0

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        thrust = catch_thrust_command(state, config, started_s, now_s)
        send_catch_setpoint(master, config, thrust)
        arm.update_state(master, state, config, timeout_s=0.0)
        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=CATCH", state, config, now_s, thrust)
            last_status_s = now_s
        time.sleep(interval_s)


def enter_altctl(master, state: arm.VehicleState, config: Config, started_s: float) -> bool:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + config.altctl_timeout_s
    last_status_s = 0.0
    last_request_s = 0.0

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        thrust = catch_thrust_command(state, config, started_s, now_s)
        send_catch_setpoint(master, config, thrust)
        arm.update_state(master, state, config, timeout_s=0.0)

        if now_s - last_request_s >= config.altctl_retry_interval_s:
            request_altctl_mode(master)
            last_request_s = now_s

        if altctl_active(state):
            print("altctl accepted")
            return True

        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=ALTCTL_WAIT", state, config, now_s, thrust)
            last_status_s = now_s

        time.sleep(interval_s)

    print("altctl denied")
    return False


def monitor_altctl(master, state: arm.VehicleState, config: Config) -> int:
    last_status_s = 0.0
    deadline_s = None if config.monitor_duration_s <= 0 else time.monotonic() + config.monitor_duration_s

    while True:
        arm.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=ALTCTL_HOLD", state, config, now_s, None)
            last_status_s = now_s

        if not state.armed:
            print("vehicle disarmed, stopping monitor")
            return 0

        if not altctl_active(state):
            print("altctl lost")
            return 1

        if deadline_s is not None and now_s >= deadline_s:
            print("monitor duration reached")
            return 0


def run(config: Config) -> int:
    master = arm.open_connection(config.connection)
    state = arm.VehicleState()
    last_status_s = 0.0

    print(f"freefall trigger: threshold={config.freefall_speed_mps:.2f}m/s duration={config.freefall_time_s:.2f}s")
    print(
        "altctl recovery: "
        f"catch_thrust={config.catch_thrust:.2f} "
        f"hover_thrust={config.hover_thrust:.2f} "
        f"vs_gain={config.vertical_speed_gain:.3f} "
        f"accel_gain={config.acceleration_guard_gain:.3f}"
    )

    while True:
        arm.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print_status("phase=WAITING", state, config, now_s, None)
            last_status_s = now_s

        if not arm.freefall_detected(state, config, now_s):
            continue

        print("trigger fired: freefall")
        started_s = time.monotonic()
        prime_and_catch(master, state, config, started_s)

        if not arm.force_arm(master, state, config):
            return 1

        print("requesting ALTCTL")
        if not enter_altctl(master, state, config, started_s):
            return 1

        print("altctl monitoring active")
        return monitor_altctl(master, state, config)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 freefall recovery script that hands off to ALTCTL.")
    parser.add_argument("--connection", default=Config.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--catch-thrust", type=float, default=Config.catch_thrust, help="Minimum thrust during the short catch phase.")
    parser.add_argument("--hover-thrust", type=float, default=Config.hover_thrust, help="Base thrust while waiting for ALTCTL.")
    parser.add_argument("--vertical-speed-gain", type=float, default=Config.vertical_speed_gain, help="Extra thrust per m/s of downward speed.")
    parser.add_argument("--acceleration-guard-gain", type=float, default=Config.acceleration_guard_gain, help="Extra thrust when downward speed is still increasing.")
    parser.add_argument("--catch-duration", type=float, default=Config.catch_duration_s, help="How long to keep manual catch active.")
    parser.add_argument("--stream-rate", type=float, default=Config.stream_rate_hz, help="Manual setpoint stream rate in Hz.")
    parser.add_argument("--prime-stream", type=float, default=Config.prime_stream_s, help="How long to stream before arming.")
    parser.add_argument("--altctl-timeout", type=float, default=Config.altctl_timeout_s, help="How long to keep retrying ALTCTL.")
    parser.add_argument("--monitor-duration", type=float, default=Config.monitor_duration_s, help="How long to monitor ALTCTL after handoff. Use 0 to monitor until disarm or mode change.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
        catch_thrust=args.catch_thrust,
        hover_thrust=args.hover_thrust,
        vertical_speed_gain=args.vertical_speed_gain,
        acceleration_guard_gain=args.acceleration_guard_gain,
        catch_duration_s=args.catch_duration,
        stream_rate_hz=args.stream_rate,
        prime_stream_s=args.prime_stream,
        altctl_timeout_s=args.altctl_timeout,
        monitor_duration_s=args.monitor_duration,
    )


def cli() -> int:
    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

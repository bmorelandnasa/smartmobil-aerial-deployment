from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import simple_arm4 as arm


@dataclass
class Config(arm.Config):
    catch_thrust: float = 0.95
    catch_duration_s: float = 1.2
    hover_thrust: float = 0.72
    min_thrust: float = 0.58
    max_thrust: float = 0.95
    altitude_gain: float = 0.025
    vertical_speed_gain: float = 0.06
    acceleration_gain: float = 0.015
    stream_rate_hz: float = 60.0
    prime_stream_s: float = 1.0
    offboard_retry_interval_s: float = 0.1
    offboard_timeout_s: float = 12.0
    rearm_retry_interval_s: float = 0.75
    arm_grace_s: float = 2.0
    handoff_altitude_window_m: float = 1.5
    handoff_vertical_speed_window_mps: float = 0.7
    handoff_ground_speed_window_mps: float = 1.5
    handoff_dwell_s: float = 1.0
    recovery_duration_s: float = 0.0
    runaway_up_speed_mps: float = 2.5
    max_thrust_warning_s: float = 1.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def velocity_age_text(state: arm.VehicleState, now_s: float) -> str:
    if state.velocity_time_s is None:
        return "n/a"
    return f"{now_s - state.velocity_time_s:.2f}s"


def velocity_fresh(state: arm.VehicleState, config: Config, now_s: float) -> bool:
    if state.velocity_time_s is None:
        return True
    return arm.velocity_is_fresh(state, config, now_s)


def altitude_fresh(state: arm.VehicleState, config: Config, now_s: float) -> bool:
    if state.altitude_time_s is None:
        return True
    return arm.altitude_is_fresh(state, config, now_s)


def effective_downward_speed(state: arm.VehicleState, config: Config, now_s: float) -> float:
    if not velocity_fresh(state, config, now_s):
        return 0.0
    return state.downward_speed_mps or 0.0


def effective_ground_speed(state: arm.VehicleState, config: Config, now_s: float) -> float:
    if not velocity_fresh(state, config, now_s):
        return 0.0
    return state.ground_speed_mps or 0.0


def vertical_accel_mps2(state: arm.VehicleState) -> float:
    history = list(state.speed_history)
    if len(history) < 2:
        return 0.0
    older = history[-2]
    newer = history[-1]
    dt = newer.time_s - older.time_s
    if dt <= 0.0:
        return 0.0
    if dt > 0.3:
        return 0.0
    return (newer.downward_speed_mps - older.downward_speed_mps) / dt


def capture_target_altitude(state: arm.VehicleState) -> tuple[Optional[float], Optional[str]]:
    return state.altitude_m, state.altitude_source


def altitude_error_m(state: arm.VehicleState, target_altitude_m: Optional[float], config: Config, now_s: float) -> float:
    if target_altitude_m is None or state.altitude_m is None or not altitude_fresh(state, config, now_s):
        return 0.0
    return target_altitude_m - state.altitude_m


def thrust_command(
    state: arm.VehicleState,
    config: Config,
    recovery_started_s: float,
    now_s: float,
    target_altitude_m: Optional[float],
) -> float:
    if not velocity_fresh(state, config, now_s):
        return config.hover_thrust

    down_speed = effective_downward_speed(state, config, now_s)
    down_accel = max(0.0, vertical_accel_mps2(state))
    alt_error = altitude_error_m(state, target_altitude_m, config, now_s)

    if now_s - recovery_started_s < config.catch_duration_s and down_speed > 0.5:
        thrust = (
            config.hover_thrust
            + max(0.0, alt_error) * config.altitude_gain
            + down_speed * config.vertical_speed_gain
            + down_accel * config.acceleration_gain
        )
        return clamp(max(thrust, config.catch_thrust), config.min_thrust, config.max_thrust)

    thrust = (
        config.hover_thrust
        + alt_error * config.altitude_gain
        + down_speed * config.vertical_speed_gain
        + down_accel * config.acceleration_gain
    )

    if alt_error < 0.0 and down_speed < -config.runaway_up_speed_mps:
        thrust -= abs(alt_error) * config.altitude_gain

    return clamp(thrust, config.min_thrust, config.max_thrust)


def send_level_setpoint(master, config: Config, thrust: float) -> None:
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


def request_altctl_mode(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        float(arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(arm.PX4_CUSTOM_MAIN_MODE_ALTCTL),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def offboard_active(state: arm.VehicleState) -> bool:
    return state.mode.upper() == "OFFBOARD"


def altctl_active(state: arm.VehicleState) -> bool:
    return state.mode.upper() in {"ALTCTL", "ALTITUDE", "ALTITUDE CONTROL"}


def hover_state(state: arm.VehicleState, target_altitude_m: Optional[float], config: Config) -> str:
    now_s = time.monotonic()
    alt_ok = abs(altitude_error_m(state, target_altitude_m, config, now_s)) <= config.handoff_altitude_window_m
    vertical_ok = state.downward_speed_mps is not None and abs(state.downward_speed_mps) <= config.handoff_vertical_speed_window_mps
    ground_ok = state.ground_speed_mps is not None and state.ground_speed_mps <= config.handoff_ground_speed_window_mps
    if state.armed and alt_ok and vertical_ok and ground_ok:
        return "HOVER_STABLE"
    if state.armed:
        return "RECOVERING"
    return "NOT_ARMED"


def armed_effective(state: arm.VehicleState, armed_latched_until_s: float, now_s: float) -> bool:
    return state.armed or now_s <= armed_latched_until_s


def handoff_ready(state: arm.VehicleState, target_altitude_m: Optional[float], config: Config) -> bool:
    now_s = time.monotonic()
    if not state.armed:
        return False
    if not offboard_active(state):
        return False
    if not velocity_fresh(state, config, now_s) or not altitude_fresh(state, config, now_s):
        return False
    if abs(altitude_error_m(state, target_altitude_m, config, now_s)) > config.handoff_altitude_window_m:
        return False
    if state.downward_speed_mps is None or abs(state.downward_speed_mps) > config.handoff_vertical_speed_window_mps:
        return False
    if state.ground_speed_mps is None or state.ground_speed_mps > config.handoff_ground_speed_window_mps:
        return False
    return True


def phase_name(stage: str) -> str:
    return f"phase={stage}"


def print_status(
    stage: str,
    state: arm.VehicleState,
    config: Config,
    now_s: float,
    target_altitude_m: Optional[float],
    thrust: Optional[float],
    armed_override: Optional[bool] = None,
) -> None:
    armed_now = state.armed if armed_override is None else armed_override
    hover_text = hover_state(state, target_altitude_m, config)
    if armed_override is True and hover_text == "NOT_ARMED":
        hover_text = "RECOVERING"
    line = (
        f"{phase_name(stage)} hover_state={hover_text} "
        f"{arm.status_line(state, config, now_s)} "
        f"armed_effective={armed_now} "
        f"target_alt_m={arm.format_number(target_altitude_m)} "
        f"alt_err_m={altitude_error_m(state, target_altitude_m, config, now_s):.2f} "
        f"vel_age={velocity_age_text(state, now_s)} "
        f"last_text={arm.recent_status_text(state, now_s)}"
    )
    if thrust is not None:
        line += f" thrust_cmd={thrust:.2f}"
    print(line)


def stream_recovery_cycle(master, state: arm.VehicleState, config: Config, recovery_started_s: float, target_altitude_m: Optional[float]) -> float:
    now_s = time.monotonic()
    thrust = thrust_command(state, config, recovery_started_s, now_s, target_altitude_m)
    send_level_setpoint(master, config, thrust)
    arm.update_state(master, state, config, timeout_s=0.0)
    return thrust


def prepare_stream(master, state: arm.VehicleState, config: Config, target_altitude_m: Optional[float]) -> None:
    interval_s = 1.0 / config.stream_rate_hz
    deadline_s = time.monotonic() + config.prime_stream_s
    started_s = time.monotonic()
    last_status_s = 0.0
    last_offboard_request_s = 0.0

    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        thrust = stream_recovery_cycle(master, state, config, started_s, target_altitude_m)
        if now_s - last_offboard_request_s >= config.offboard_retry_interval_s:
            request_offboard_mode(master)
            last_offboard_request_s = now_s
        if now_s - last_status_s >= config.status_interval_s:
            print_status("PREPARE", state, config, now_s, target_altitude_m, thrust)
            last_status_s = now_s
        time.sleep(interval_s)


def recover_and_handoff(master, state: arm.VehicleState, config: Config, target_altitude_m: Optional[float]) -> int:
    interval_s = 1.0 / config.stream_rate_hz
    recovery_started_s = time.monotonic()
    offboard_deadline_s = recovery_started_s + config.offboard_timeout_s
    recovery_deadline_s = None if config.recovery_duration_s <= 0 else recovery_started_s + config.recovery_duration_s
    last_status_s = 0.0
    last_offboard_request_s = 0.0
    stable_since_s: Optional[float] = None
    last_max_thrust_s: Optional[float] = None
    last_rearm_attempt_s: Optional[float] = None
    altctl_requested = False
    armed_latched_until_s = recovery_started_s + config.arm_grace_s
    target_source = state.altitude_source

    while True:
        now_s = time.monotonic()
        thrust = stream_recovery_cycle(master, state, config, recovery_started_s, target_altitude_m)

        if target_source != "LOCAL_POSITION_NED" and state.altitude_source == "LOCAL_POSITION_NED" and altitude_fresh(state, config, now_s):
            target_altitude_m = state.altitude_m
            target_source = state.altitude_source
            print(f"target altitude recaptured from {target_source}: {arm.format_number(target_altitude_m)} m")

        if state.armed:
            armed_latched_until_s = now_s + config.arm_grace_s

        if thrust >= config.max_thrust - 1e-6:
            last_max_thrust_s = last_max_thrust_s or now_s
        else:
            last_max_thrust_s = None

        if not altctl_requested and now_s - last_offboard_request_s >= config.offboard_retry_interval_s:
            request_offboard_mode(master)
            last_offboard_request_s = now_s

        if handoff_ready(state, target_altitude_m, config):
            stable_since_s = stable_since_s or now_s
        else:
            stable_since_s = None

        if stable_since_s is not None and now_s - stable_since_s >= config.handoff_dwell_s:
            request_altctl_mode(master)
            altctl_requested = True

        if altctl_requested and altctl_active(state):
            print("altctl accepted")
            return monitor_altctl(master, state, config, target_altitude_m)

        if not altctl_requested and now_s >= offboard_deadline_s and not offboard_active(state):
            print("offboard not confirmed yet, still streaming and retrying")
            offboard_deadline_s = now_s + 1.0

        if last_max_thrust_s is not None and now_s - last_max_thrust_s >= config.max_thrust_warning_s:
            if effective_downward_speed(state, config, now_s) > config.handoff_vertical_speed_window_mps:
                print("warning: thrust saturated at max and descent is not yet under control")
                last_max_thrust_s = now_s

        if now_s - last_status_s >= config.status_interval_s:
            if altctl_requested:
                stage = "ALTCTL_REQUEST"
            elif now_s - recovery_started_s < config.catch_duration_s:
                stage = "CATCH"
            else:
                stage = "STABILIZE"
            print_status(
                stage,
                state,
                config,
                now_s,
                target_altitude_m,
                thrust,
                armed_override=armed_effective(state, armed_latched_until_s, now_s),
            )
            last_status_s = now_s

        if not armed_effective(state, armed_latched_until_s, now_s):
            within_grace = now_s - recovery_started_s <= config.arm_grace_s
            still_falling = effective_downward_speed(state, config, now_s) >= config.freefall_speed_mps * 0.5
            if within_grace or still_falling:
                if last_rearm_attempt_s is None or now_s - last_rearm_attempt_s >= config.rearm_retry_interval_s:
                    print("vehicle not armed yet, retrying force-arm while streaming")
                    if arm.force_arm(master, state, config):
                        armed_latched_until_s = time.monotonic() + config.arm_grace_s
                    last_rearm_attempt_s = now_s
            else:
                print("vehicle disarmed, stopping recovery stream")
                return 1

        if recovery_deadline_s is not None and now_s >= recovery_deadline_s:
            print("recovery duration reached, stopping recovery stream")
            return 0

        time.sleep(interval_s)


def monitor_altctl(master, state: arm.VehicleState, config: Config, target_altitude_m: Optional[float]) -> int:
    last_status_s = 0.0
    deadline_s = None if config.recovery_duration_s <= 0 else time.monotonic() + config.recovery_duration_s

    while True:
        arm.update_state(master, state, config, timeout_s=0.1)
        now_s = time.monotonic()

        if now_s - last_status_s >= config.status_interval_s:
            print_status("ALTCTL_HOLD", state, config, now_s, target_altitude_m, None)
            last_status_s = now_s

        if not state.armed:
            print("vehicle disarmed, stopping monitor")
            return 0

        if not altctl_active(state):
            print("altctl lost")
            return 1

        if deadline_s is not None and now_s >= deadline_s:
            print("recovery duration reached, stopping monitor")
            return 0


def run(config: Config) -> int:
    master = arm.open_connection(config.connection)
    state = arm.VehicleState()
    last_status_s = 0.0

    print(f"freefall trigger: threshold={config.freefall_speed_mps:.2f}m/s duration={config.freefall_time_s:.2f}s")
    print(
        "px4 setup assumptions: "
        "COM_OF_LOSS_T long enough for recovery, "
        "offboard-loss action not immediately unusable, "
        "land/disarm logic not auto-disarming immediately after midair re-arm"
    )
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
            print_status("WAITING", state, config, now_s, None, None)
            last_status_s = now_s

        if not arm.freefall_detected(state, config, now_s):
            continue

        print("trigger fired: freefall")
        target_altitude_m, target_source = capture_target_altitude(state)
        print(f"target altitude captured: {arm.format_number(target_altitude_m)} m source={target_source or 'none'}")

        prepare_stream(master, state, config, target_altitude_m)

        if state.altitude_source == "LOCAL_POSITION_NED" and arm.altitude_is_fresh(state, config, time.monotonic()):
            target_altitude_m = state.altitude_m
            print(f"target altitude refreshed before arm: {arm.format_number(target_altitude_m)} m source={state.altitude_source}")

        print("phase=ARMING")
        if not arm.force_arm(master, state, config):
            return 1

        print("recovery streaming active")
        return recover_and_handoff(master, state, config, target_altitude_m)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PX4 offboard recovery with ALTCTL handoff.")
    parser.add_argument("--connection", default=Config.connection, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s, help="Required freefall duration in seconds.")
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--catch-thrust", type=float, default=Config.catch_thrust, help="Minimum thrust during the catch phase.")
    parser.add_argument("--catch-duration", type=float, default=Config.catch_duration_s, help="How long to keep the stronger catch behavior.")
    parser.add_argument("--hover-thrust", type=float, default=Config.hover_thrust, help="Base thrust after the catch phase.")
    parser.add_argument("--min-thrust", type=float, default=Config.min_thrust, help="Minimum commanded thrust.")
    parser.add_argument("--max-thrust", type=float, default=Config.max_thrust, help="Maximum commanded thrust.")
    parser.add_argument("--altitude-gain", type=float, default=Config.altitude_gain, help="Extra thrust per meter below target altitude.")
    parser.add_argument("--vertical-speed-gain", type=float, default=Config.vertical_speed_gain, help="Extra thrust per m/s of downward speed.")
    parser.add_argument("--acceleration-gain", type=float, default=Config.acceleration_gain, help="Extra thrust when downward speed is still increasing.")
    parser.add_argument("--stream-rate", type=float, default=Config.stream_rate_hz, help="Setpoint stream rate in Hz.")
    parser.add_argument("--prime-stream", type=float, default=Config.prime_stream_s, help="How long to stream before arming.")
    parser.add_argument("--offboard-retry-interval", type=float, default=Config.offboard_retry_interval_s, help="How often to re-request Offboard.")
    parser.add_argument("--offboard-timeout", type=float, default=Config.offboard_timeout_s, help="How long to wait before warning that Offboard is still not confirmed.")
    parser.add_argument("--rearm-retry-interval", type=float, default=Config.rearm_retry_interval_s, help="How often to retry force-arm if PX4 drops back to disarmed during the catch.")
    parser.add_argument("--arm-grace", type=float, default=Config.arm_grace_s, help="How long to tolerate and retry an unarmed state right after recovery starts.")
    parser.add_argument("--handoff-altitude-window", type=float, default=Config.handoff_altitude_window_m, help="Altitude error window for ALTCTL handoff.")
    parser.add_argument("--handoff-vertical-speed-window", type=float, default=Config.handoff_vertical_speed_window_mps, help="Vertical speed window for ALTCTL handoff.")
    parser.add_argument("--handoff-ground-speed-window", type=float, default=Config.handoff_ground_speed_window_mps, help="Ground speed window for ALTCTL handoff.")
    parser.add_argument("--handoff-dwell", type=float, default=Config.handoff_dwell_s, help="How long stability must be held before ALTCTL request.")
    parser.add_argument("--recovery-duration", type=float, default=Config.recovery_duration_s, help="How long to keep recovery or monitor active. Use 0 to hold until disarm/manual takeover.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        status_interval_s=args.status_interval,
        catch_thrust=args.catch_thrust,
        catch_duration_s=args.catch_duration,
        hover_thrust=args.hover_thrust,
        min_thrust=args.min_thrust,
        max_thrust=args.max_thrust,
        altitude_gain=args.altitude_gain,
        vertical_speed_gain=args.vertical_speed_gain,
        acceleration_gain=args.acceleration_gain,
        stream_rate_hz=args.stream_rate,
        prime_stream_s=args.prime_stream,
        offboard_retry_interval_s=args.offboard_retry_interval,
        offboard_timeout_s=args.offboard_timeout,
        rearm_retry_interval_s=args.rearm_retry_interval,
        arm_grace_s=args.arm_grace,
        handoff_altitude_window_m=args.handoff_altitude_window,
        handoff_vertical_speed_window_mps=args.handoff_vertical_speed_window,
        handoff_ground_speed_window_mps=args.handoff_ground_speed_window,
        handoff_dwell_s=args.handoff_dwell,
        recovery_duration_s=args.recovery_duration,
    )


def cli() -> int:
    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

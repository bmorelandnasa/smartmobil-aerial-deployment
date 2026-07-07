"""Freefall recovery script that catches with Offboard thrust, then hands off to PX4 Hold."""

from __future__ import annotations

import argparse
import datetime as dt
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    from simple_pid import PID
except ImportError:
    PID = None

import simple_arm3 as arm


@dataclass
class Config(arm.Config):
    """All tunable recovery settings exposed through the CLI."""

    stream_rate_hz: float = 50.0
    recovery_thrust: float = 0.55
    min_thrust: float = 0.20
    max_thrust: float = 0.88
    catch_thrust: float = 0.90
    catch_duration_s: float = 0.25
    target_vz_mps: float = 0.0
    pid_kp: float = 0.045
    pid_ki: float = 0.0
    pid_kd: float = 0.025
    recovery_duration_s: float = 10.0
    offboard_retry_interval_s: float = 0.2
    arm_retry_interval_s: float = 0.2
    force_arm_burst_s: float = 2.0
    tilt_moderate_deg: float = 35.0
    tilt_bad_deg: float = 60.0
    tilt_inverted_deg: float = 90.0
    tilt_moderate_thrust_cap: float = 0.55
    tilt_bad_thrust_cap: float = 0.35
    tilt_inverted_thrust_cap: float = 0.15
    handoff_max_tilt_deg: float = 25.0
    handoff_max_rate_rad_s: float = 1.5
    simulated_tilt_deg: Optional[float] = None
    handoff_hold: bool = True
    handoff_after_offboard_s: float = 0.5
    handoff_vz_mps: float = 3.5
    handoff_climb_limit_mps: float = -0.5
    handoff_dwell_s: float = 0.0
    handoff_min_altitude_m: float = 10.0
    handoff_timeout_s: float = 2.0
    handoff_retry_interval_s: float = 0.3
    log_dir: str = "logs"
    no_log: bool = False
    mc_initial_local_position_delay_s: float = 0.0
    mc_heartbeat_gap_s: float = 0.0
    mc_freefall_detection_delay_s: float = 0.0


class Tee:
    """Write console output to both the terminal and a log file."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text: str) -> int:
        for stream in self.streams:
            stream.write(text)
            stream.flush()
        return len(text)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


def enable_logging(config: Config) -> None:
    """Mirror stdout/stderr into a timestamped log file unless disabled."""

    if config.no_log:
        return
    log_dir = Path(config.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    path = log_dir / f"simple_hover3_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    log_file = path.open("w", encoding="utf-8", buffering=1)
    sys.stdout = Tee(sys.__stdout__, log_file)
    sys.stderr = Tee(sys.__stderr__, log_file)
    print(f"log file: {path.resolve()}")


def request_offboard(master) -> None:
    """Ask PX4 to enter Offboard mode so our thrust setpoints are used."""

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        float(arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(arm.PX4_CUSTOM_MAIN_MODE_OFFBOARD),
        0,
        0,
        0,
        0,
        0,
    )


def request_hold(master) -> None:
    """Ask PX4 to enter Hold/Loiter mode after the emergency catch is stable enough."""

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_DO_SET_MODE,
        0,
        float(arm.MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(arm.PX4_CUSTOM_MAIN_MODE_AUTO),
        float(arm.PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        0,
        0,
        0,
        0,
    )


def send_level_thrust(master, thrust: float) -> None:
    """Send a level-attitude thrust setpoint for PX4 Offboard control."""

    master.mav.set_attitude_target_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        1 | 2 | 4,
        [1.0, 0.0, 0.0, 0.0],
        0,
        0,
        0,
        thrust,
    )


def offboard_active(state: arm.VehicleState) -> bool:
    """Return True when PX4 reports Offboard mode."""

    return state.mode.upper() == "OFFBOARD"


def hold_active(state: arm.VehicleState) -> bool:
    """Return True when PX4 reports a hold/loiter style mode."""

    mode = state.mode.upper()
    return mode in {"AUTO.LOITER", "HOLD", "LOITER"}


def handoff_ready(
    state: arm.VehicleState,
    config: Config,
    recovery_started_s: float,
    now_s: float,
) -> bool:
    """Return True when recovery has slowed the vehicle enough to request PX4 Hold."""

    if not config.handoff_hold:
        return False
    if now_s - recovery_started_s < config.handoff_after_offboard_s:
        return False
    if not (state.armed or arm_accepted(state)):
        return False
    if (
        state.downward_speed_mps is None
        or state.altitude_m is None
        or not arm.velocity_fresh(state, config, now_s)
        or not arm.altitude_fresh(state, config, now_s)
    ):
        return False
    tilt = effective_tilt_deg(state, config, now_s)
    body_rate_fn = getattr(arm, "max_body_rate_rad_s", None)
    body_rate = body_rate_fn(state) if body_rate_fn is not None else None
    if tilt is not None and tilt > config.handoff_max_tilt_deg:
        return False
    if body_rate is not None and body_rate > config.handoff_max_rate_rad_s:
        return False
    return (
        config.handoff_climb_limit_mps <= state.downward_speed_mps <= config.handoff_vz_mps
        and state.altitude_m >= config.handoff_min_altitude_m
    )


def ack_text(state: arm.VehicleState) -> str:
    """Format the newest mode and arm command acknowledgements for logs."""

    mode_ack = state.command_acks.get(arm.MAVLINK.MAV_CMD_DO_SET_MODE, "n/a")
    arm_ack = state.command_acks.get(arm.MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM, "n/a")
    return f"ack(mode={mode_ack},arm={arm_ack})"


def arm_accepted(state: arm.VehicleState) -> bool:
    """Return True once PX4 has accepted our arm command."""

    return state.command_acks.get(arm.MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM) == arm.MAVLINK.MAV_RESULT_ACCEPTED


def mode_ack_accepted_after(state: arm.VehicleState, command_time_s: Optional[float]) -> bool:
    """Return True when a mode command ACK was accepted after the given request time."""

    if command_time_s is None:
        return False
    return (
        state.command_acks.get(arm.MAVLINK.MAV_CMD_DO_SET_MODE) == arm.MAVLINK.MAV_RESULT_ACCEPTED
        and state.command_ack_times_s.get(arm.MAVLINK.MAV_CMD_DO_SET_MODE, 0.0) >= command_time_s
    )


def hold_heartbeat_after(state: arm.VehicleState, command_time_s: Optional[float]) -> bool:
    """Return True when a fresh post-request heartbeat reports Hold/Loiter."""

    if command_time_s is None or state.heartbeat_time_s is None:
        return False
    return state.heartbeat_time_s >= command_time_s and hold_active(state)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric value into a safe range."""

    return max(low, min(high, value))


def effective_tilt_deg(state: arm.VehicleState, config: Config, now_s: float) -> Optional[float]:
    """Return real tilt, or a simulated tilt used only for bench-testing the gate."""

    if config.simulated_tilt_deg is not None:
        return config.simulated_tilt_deg
    if not arm.attitude_fresh(state, config, now_s):
        return None
    return arm.tilt_deg(state)


def attitude_thrust_cap(state: arm.VehicleState, config: Config, now_s: float) -> tuple[float, str]:
    """Limit thrust while PX4 is still rotating the vehicle back toward level."""

    tilt = effective_tilt_deg(state, config, now_s)
    if tilt is None:
        return config.max_thrust, "NO_ATTITUDE_LIMIT"
    if tilt >= config.tilt_inverted_deg:
        return config.tilt_inverted_thrust_cap, "LEVEL_INVERTED"
    if tilt >= config.tilt_bad_deg:
        return config.tilt_bad_thrust_cap, "LEVEL_BAD_TILT"
    if tilt >= config.tilt_moderate_deg:
        return config.tilt_moderate_thrust_cap, "LEVEL_LIMITED_TILT"
    return config.max_thrust, "VERTICAL_RECOVERY"


def apply_initial_local_position_delay(
    state: arm.VehicleState,
    config: Config,
    phase_started_s: float,
    now_s: float,
) -> None:
    """Hide early local-position samples for Monte Carlo timing tests."""

    if config.mc_initial_local_position_delay_s <= 0.0:
        return
    if now_s - phase_started_s > config.mc_initial_local_position_delay_s:
        return
    state.altitude_m = None
    state.altitude_time_s = None
    state.downward_speed_mps = None
    state.velocity_time_s = None
    state.speed_history.clear()


def apply_heartbeat_gap(
    state: arm.VehicleState,
    config: Config,
    phase_started_s: float,
    now_s: float,
) -> None:
    """Hide an early heartbeat window for Monte Carlo timing tests."""

    if config.mc_heartbeat_gap_s <= 0.0:
        return
    if now_s - phase_started_s <= config.mc_heartbeat_gap_s:
        state.heartbeat_time_s = None


def print_status(
    prefix: str,
    state: arm.VehicleState,
    config: Config,
    thrust: Optional[float] = None,
    pid_output: Optional[float] = None,
    reason: Optional[str] = None,
) -> None:
    """Print one readable status line for wait, recovery, and handoff phases."""

    now_s = time.monotonic()
    accel = arm.vertical_accel_mps2(state)
    line = f"{prefix} {arm.status_line(state, config, now_s)} {ack_text(state)}"
    line += f" acc={arm.fmt(accel)}m/s2"
    if thrust is not None:
        line += f" thrust={thrust:.2f}"
    if pid_output is not None:
        line += f" pid={pid_output:.2f} target_vz={config.target_vz_mps:.1f}m/s"
    if reason is not None:
        line += f" reason={reason}"
    px4_text = arm.recent_status_text(state, now_s)
    if px4_text != "none":
        line += f" px4='{px4_text}'"
    print(line)


def build_pid(config: Config):
    """Build the SimplePID controller that adjusts thrust from vertical speed."""

    if PID is None:
        raise RuntimeError("simple-pid is not installed. Run: pip install -r requirements.txt")
    pid = PID(config.pid_kp, config.pid_ki, config.pid_kd, setpoint=config.target_vz_mps)
    pid.output_limits = (config.min_thrust - config.recovery_thrust, config.max_thrust - config.recovery_thrust)
    pid.sample_time = 0.0
    return pid


def thrust_command(state: arm.VehicleState, config: Config, pid, started_s: float, now_s: float) -> tuple[float, float, str]:
    """Convert vertical speed into a thrust command for the current recovery cycle."""

    if not arm.velocity_fresh(state, config, now_s) or state.downward_speed_mps is None:
        thrust = config.catch_thrust
        cap, reason = attitude_thrust_cap(state, config, now_s)
        thrust = min(thrust, cap)
        return thrust, thrust - config.recovery_thrust, reason

    if state.downward_speed_mps < -0.3:
        pid.reset()
        return config.min_thrust, config.min_thrust - config.recovery_thrust, "CUT_REBOUND"

    pid_output = float(pid(-(state.downward_speed_mps)))
    thrust = clamp(config.recovery_thrust + pid_output, config.min_thrust, config.max_thrust)
    if now_s - started_s <= config.catch_duration_s and state.downward_speed_mps > 0.5:
        thrust = max(thrust, config.catch_thrust)
    cap, reason = attitude_thrust_cap(state, config, now_s)
    thrust = min(thrust, cap)
    return thrust, thrust - config.recovery_thrust, reason


def recovery_loop(master, state: arm.VehicleState, config: Config) -> int:
    """Main emergency loop: stream thrust, force-arm, request Offboard, then hand off."""

    interval_s = 1.0 / config.stream_rate_hz
    started_s = time.monotonic()
    pid = build_pid(config)
    last_status_s = 0.0
    last_offboard_s = 0.0
    last_arm_s = 0.0
    handoff_requested_s: Optional[float] = None
    handoff_ready_since_s: Optional[float] = None
    last_handoff_s = 0.0

    while time.monotonic() - started_s < config.recovery_duration_s:
        cycle_s = time.monotonic()

        # 1) Read fresh telemetry before computing the next command.
        arm.update_state(master, state, config, timeout_s=min(0.01, interval_s * 0.5))
        now_s = time.monotonic()
        apply_heartbeat_gap(state, config, started_s, now_s)

        if handoff_requested_s is not None and mode_ack_accepted_after(state, handoff_requested_s):
            print_status("[handoff] HOLD command accepted; ending active recovery", state, config)
            return 0

        if handoff_requested_s is not None and hold_heartbeat_after(state, handoff_requested_s):
            print_status("[handoff] HOLD heartbeat confirmed", state, config)
            return 0

        if handoff_requested_s is not None and now_s - handoff_requested_s >= config.handoff_timeout_s:
            print_status("[handoff] HOLD not heartbeat-confirmed yet; continuing recovery", state, config)
            handoff_requested_s = None

        # 2) Compute and stream thrust every cycle so PX4 always has a fresh setpoint.
        thrust, pid_output, reason = thrust_command(state, config, pid, started_s, now_s)
        send_level_thrust(master, thrust)

        # 3) Request Hold only after Offboard recovery has slowed the vehicle enough.
        if handoff_ready(state, config, started_s, now_s):
            handoff_ready_since_s = handoff_ready_since_s or now_s
        else:
            handoff_ready_since_s = None

        if (
            handoff_requested_s is None
            and handoff_ready_since_s is not None
            and now_s - handoff_ready_since_s >= config.handoff_dwell_s
            and now_s - last_handoff_s >= config.handoff_retry_interval_s
        ):
            print_status("[handoff] requesting HOLD", state, config, thrust, pid_output, reason)
            state.command_acks.pop(arm.MAVLINK.MAV_CMD_DO_SET_MODE, None)
            state.command_ack_times_s.pop(arm.MAVLINK.MAV_CMD_DO_SET_MODE, None)
            request_hold(master)
            handoff_requested_s = now_s
            last_handoff_s = now_s

        # 4) During the emergency window, keep retrying Offboard and force-arm.
        in_initial_burst = now_s - started_s <= config.force_arm_burst_s
        if (
            handoff_requested_s is None
            and now_s - last_offboard_s >= config.offboard_retry_interval_s
            and (in_initial_burst or not offboard_active(state))
        ):
            request_offboard(master)
            last_offboard_s = now_s
        if now_s - last_arm_s >= config.arm_retry_interval_s and (in_initial_burst or not state.armed):
            arm.send_arm_command(master, force=True)
            last_arm_s = now_s

        # 5) Print a readable summary at human speed while the loop runs at stream_rate_hz.
        if now_s - last_status_s >= config.status_interval_s:
            print_status("[recover]", state, config, thrust, pid_output, reason)
            last_status_s = now_s

        time.sleep(max(0.0, interval_s - (time.monotonic() - cycle_s)))

    if handoff_requested_s is not None:
        print_status("[done] recovery timeout while waiting for HOLD heartbeat confirmation", state, config)
    else:
        print_status("[done]", state, config)
    return 0


def run(config: Config) -> int:
    """Connect, wait for freefall, and start the recovery loop."""

    enable_logging(config)
    master = arm.open_connection(config.connection, config.connection_timeout_s)
    arm.request_basic_telemetry(master)
    state = arm.VehicleState()
    wait_started_s = time.monotonic()
    freefall_seen_s: Optional[float] = None
    last_status_s = 0.0
    print(
        f"[setup] trigger={config.freefall_speed_mps:.1f}m/s for {config.freefall_time_s:.2f}s "
        f"stream={config.stream_rate_hz:.0f}Hz base_thrust={config.recovery_thrust:.2f} "
        f"pid=({config.pid_kp:.3f},{config.pid_ki:.3f},{config.pid_kd:.3f}) "
        f"tilt_caps={config.tilt_moderate_deg:.0f}/{config.tilt_bad_deg:.0f}/{config.tilt_inverted_deg:.0f}deg "
        f"handoff={'hold' if config.handoff_hold else 'off'} "
        f"handoff_after_offboard={config.handoff_after_offboard_s:.2f}s "
        f"handoff_vz=[{config.handoff_climb_limit_mps:.1f},{config.handoff_vz_mps:.1f}]m/s "
        f"duration={config.recovery_duration_s:.1f}s"
    )

    while True:
        send_level_thrust(master, config.recovery_thrust)
        arm.update_state(master, state, config, timeout_s=0.05)
        now_s = time.monotonic()
        apply_initial_local_position_delay(state, config, wait_started_s, now_s)
        if now_s - last_status_s >= config.status_interval_s:
            print_status("[wait]", state, config)
            last_status_s = now_s
        if arm.freefall_detected(state, config, now_s):
            freefall_seen_s = freefall_seen_s or now_s
            if now_s - freefall_seen_s >= config.mc_freefall_detection_delay_s:
                print_status("[drop]", state, config)
                return recovery_loop(master, state, config)
        else:
            freefall_seen_s = None


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the CLI for tuning the recovery script."""

    parser = argparse.ArgumentParser(description="Minimal PX4 Offboard freefall recovery.")
    parser.add_argument("--connection", default=Config.connection)
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_mps)
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_s)
    parser.add_argument("--history-window", type=float, default=Config.history_window_s)
    parser.add_argument("--status-interval", type=float, default=Config.status_interval_s)
    parser.add_argument("--stream-rate", type=float, default=Config.stream_rate_hz)
    parser.add_argument("--recovery-thrust", "--base-thrust", type=float, default=Config.recovery_thrust)
    parser.add_argument("--min-thrust", type=float, default=Config.min_thrust)
    parser.add_argument("--max-thrust", type=float, default=Config.max_thrust)
    parser.add_argument("--catch-thrust", type=float, default=Config.catch_thrust)
    parser.add_argument("--catch-duration", type=float, default=Config.catch_duration_s)
    parser.add_argument("--target-vz", type=float, default=Config.target_vz_mps)
    parser.add_argument("--pid-kp", type=float, default=Config.pid_kp)
    parser.add_argument("--pid-ki", type=float, default=Config.pid_ki)
    parser.add_argument("--pid-kd", type=float, default=Config.pid_kd)
    parser.add_argument("--recovery-duration", type=float, default=Config.recovery_duration_s)
    parser.add_argument("--offboard-retry-interval", type=float, default=Config.offboard_retry_interval_s)
    parser.add_argument("--arm-retry-interval", type=float, default=Config.arm_retry_interval_s)
    parser.add_argument("--force-arm-burst", type=float, default=Config.force_arm_burst_s)
    parser.add_argument("--tilt-moderate", type=float, default=Config.tilt_moderate_deg)
    parser.add_argument("--tilt-bad", type=float, default=Config.tilt_bad_deg)
    parser.add_argument("--tilt-inverted", type=float, default=Config.tilt_inverted_deg)
    parser.add_argument("--tilt-moderate-thrust-cap", type=float, default=Config.tilt_moderate_thrust_cap)
    parser.add_argument("--tilt-bad-thrust-cap", type=float, default=Config.tilt_bad_thrust_cap)
    parser.add_argument("--tilt-inverted-thrust-cap", type=float, default=Config.tilt_inverted_thrust_cap)
    parser.add_argument("--handoff-max-tilt", type=float, default=Config.handoff_max_tilt_deg)
    parser.add_argument("--handoff-max-rate", type=float, default=Config.handoff_max_rate_rad_s)
    parser.add_argument("--simulated-tilt-deg", type=float, default=None)
    parser.add_argument("--no-handoff-hold", action="store_true")
    parser.add_argument("--handoff-after-offboard", type=float, default=Config.handoff_after_offboard_s)
    parser.add_argument("--handoff-vz", type=float, default=Config.handoff_vz_mps)
    parser.add_argument("--handoff-climb-limit", type=float, default=Config.handoff_climb_limit_mps)
    parser.add_argument("--handoff-dwell", type=float, default=Config.handoff_dwell_s)
    parser.add_argument("--handoff-min-altitude", type=float, default=Config.handoff_min_altitude_m)
    parser.add_argument("--handoff-timeout", type=float, default=Config.handoff_timeout_s)
    parser.add_argument("--handoff-retry-interval", type=float, default=Config.handoff_retry_interval_s)
    parser.add_argument("--log-dir", default=Config.log_dir)
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--mc-initial-local-position-delay", type=float, default=Config.mc_initial_local_position_delay_s)
    parser.add_argument("--mc-heartbeat-gap", type=float, default=Config.mc_heartbeat_gap_s)
    parser.add_argument("--mc-freefall-detection-delay", type=float, default=Config.mc_freefall_detection_delay_s)
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    """Convert parsed CLI arguments into Config."""

    return Config(
        connection=args.connection,
        freefall_speed_mps=args.freefall_speed,
        freefall_time_s=args.freefall_time,
        history_window_s=args.history_window,
        status_interval_s=args.status_interval,
        stream_rate_hz=args.stream_rate,
        recovery_thrust=args.recovery_thrust,
        min_thrust=args.min_thrust,
        max_thrust=args.max_thrust,
        catch_thrust=args.catch_thrust,
        catch_duration_s=args.catch_duration,
        target_vz_mps=args.target_vz,
        pid_kp=args.pid_kp,
        pid_ki=args.pid_ki,
        pid_kd=args.pid_kd,
        recovery_duration_s=args.recovery_duration,
        offboard_retry_interval_s=args.offboard_retry_interval,
        arm_retry_interval_s=args.arm_retry_interval,
        force_arm_burst_s=args.force_arm_burst,
        tilt_moderate_deg=args.tilt_moderate,
        tilt_bad_deg=args.tilt_bad,
        tilt_inverted_deg=args.tilt_inverted,
        tilt_moderate_thrust_cap=args.tilt_moderate_thrust_cap,
        tilt_bad_thrust_cap=args.tilt_bad_thrust_cap,
        tilt_inverted_thrust_cap=args.tilt_inverted_thrust_cap,
        handoff_max_tilt_deg=args.handoff_max_tilt,
        handoff_max_rate_rad_s=args.handoff_max_rate,
        simulated_tilt_deg=args.simulated_tilt_deg,
        handoff_hold=not args.no_handoff_hold,
        handoff_after_offboard_s=args.handoff_after_offboard,
        handoff_vz_mps=args.handoff_vz,
        handoff_climb_limit_mps=args.handoff_climb_limit,
        handoff_dwell_s=args.handoff_dwell,
        handoff_min_altitude_m=args.handoff_min_altitude,
        handoff_timeout_s=args.handoff_timeout,
        handoff_retry_interval_s=args.handoff_retry_interval,
        log_dir=args.log_dir,
        no_log=args.no_log,
        mc_initial_local_position_delay_s=args.mc_initial_local_position_delay,
        mc_heartbeat_gap_s=args.mc_heartbeat_gap,
        mc_freefall_detection_delay_s=args.mc_freefall_detection_delay,
    )


def cli() -> int:
    """Command-line entry point."""

    return run(config_from_args(build_arg_parser().parse_args()))


if __name__ == "__main__":
    raise SystemExit(cli())

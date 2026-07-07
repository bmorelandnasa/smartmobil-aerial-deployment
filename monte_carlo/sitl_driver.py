from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
import simple_arm3 as arm

from .log_parser import parse_latest_recovery_log
from .sampler import SamplerConfig, make_trial_sample, trial_seeds


MAV_FRAME_LOCAL_NED = getattr(arm.MAVLINK, "MAV_FRAME_LOCAL_NED", 1)
POSITION_TARGET_TYPEMASK_POSITION_ONLY = (
    getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_VX_IGNORE", 8)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_VY_IGNORE", 16)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_VZ_IGNORE", 32)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_AX_IGNORE", 64)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_AY_IGNORE", 128)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_AZ_IGNORE", 256)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_YAW_IGNORE", 1024)
    | getattr(arm.MAVLINK, "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE", 2048)
)


def request_offboard(master) -> None:
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


def send_position_target(master, altitude_m: float) -> None:
    """Stream a local-NED position target for the setup climb."""

    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        MAV_FRAME_LOCAL_NED,
        POSITION_TARGET_TYPEMASK_POSITION_ONLY,
        0.0,
        0.0,
        -altitude_m,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def command_force_disarm(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        arm.MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        arm.FORCE_ARM_MAGIC,
        0,
        0,
        0,
        0,
        0,
    )


def wait_for_altitude(
    master,
    state: arm.VehicleState,
    config: arm.Config,
    target_m: float,
    timeout_s: float,
    log,
) -> bool:
    deadline_s = time.monotonic() + timeout_s
    last_mode_request_s = 0.0
    last_arm_request_s = 0.0
    last_log_s = 0.0

    while time.monotonic() < deadline_s:
        cycle_s = time.monotonic()
        send_position_target(master, target_m)
        arm.update_state(master, state, config, timeout_s=0.02)
        now_s = time.monotonic()

        if now_s - last_mode_request_s >= 0.5:
            request_offboard(master)
            last_mode_request_s = now_s
        if now_s - last_arm_request_s >= 0.5 and not state.armed:
            arm.send_arm_command(master, force=True)
            last_arm_request_s = now_s

        if now_s - last_log_s >= 1.0:
            print(
                f"[supervisor] setup alt={arm.fmt(state.altitude_m)}m "
                f"target={target_m:.1f}m mode={state.mode} armed={state.armed}",
                file=log,
                flush=True,
            )
            last_log_s = now_s

        if state.altitude_m is not None and state.altitude_m >= target_m * 0.90 and arm.altitude_fresh(state, config, now_s):
            return True
        time.sleep(max(0.0, 0.05 - (time.monotonic() - cycle_s)))
    return False


def prepare_drop(master, state: arm.VehicleState, config: arm.Config, altitude_m: float, timeout_s: float, log) -> bool:
    print(f"[supervisor] streaming Offboard climb target {altitude_m:.1f}m", file=log, flush=True)

    warmup_deadline_s = time.monotonic() + 1.0
    while time.monotonic() < warmup_deadline_s:
        send_position_target(master, altitude_m)
        arm.update_state(master, state, config, timeout_s=0.02)
        time.sleep(0.05)

    request_offboard(master)
    arm.send_arm_command(master, force=True)
    reached = wait_for_altitude(master, state, config, altitude_m, timeout_s, log)
    if reached:
        print(f"[supervisor] reached drop altitude: {state.altitude_m:.1f}m", file=log, flush=True)
    else:
        print(f"[supervisor] takeoff altitude timeout; latest altitude={arm.fmt(state.altitude_m)}m", file=log, flush=True)
    return reached


def build_recovery_command(args: argparse.Namespace, trial_dir: Path, sample) -> list[str]:
    return [
        sys.executable,
        "simple_hover3.py",
        "--connection",
        args.recovery_connection,
        "--log-dir",
        str(trial_dir),
        "--recovery-thrust",
        f"{sample.recovery_thrust:.6f}",
        "--pid-kp",
        f"{sample.pid_kp:.6f}",
        "--pid-ki",
        f"{sample.pid_ki:.6f}",
        "--pid-kd",
        f"{sample.pid_kd:.6f}",
        "--catch-duration",
        f"{sample.catch_duration_s:.6f}",
        "--tilt-moderate",
        f"{sample.tilt_moderate_deg:.6f}",
        "--tilt-bad",
        f"{sample.tilt_bad_deg:.6f}",
        "--tilt-inverted",
        f"{sample.tilt_inverted_deg:.6f}",
        "--handoff-after-offboard",
        f"{sample.handoff_after_offboard_s:.6f}",
        "--handoff-timeout",
        f"{sample.handoff_timeout_s:.6f}",
        "--history-window",
        f"{sample.history_window_s:.6f}",
        "--mc-initial-local-position-delay",
        f"{sample.initial_local_position_delay_s:.6f}",
        "--mc-heartbeat-gap",
        f"{sample.heartbeat_gap_s:.6f}",
        "--mc-freefall-detection-delay",
        f"{sample.freefall_detection_delay_s:.6f}",
    ]


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_trial(args: argparse.Namespace, session_dir: Path, master, state: arm.VehicleState, supervisor_config: arm.Config, index: int, seed: int):
    sample = make_trial_sample(index, seed, SamplerConfig(min_drop_altitude_m=args.min_drop_altitude, max_drop_altitude_m=args.max_drop_altitude))
    trial_dir = session_dir / f"trial_{index:04d}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    write_json(trial_dir / "trial_config.json", sample.to_dict())

    with (trial_dir / "supervisor.log").open("w", encoding="utf-8", buffering=1) as log:
        print(f"[supervisor] trial={index} seed={seed}", file=log, flush=True)
        print(f"[supervisor] sample={json.dumps(sample.to_dict(), sort_keys=True)}", file=log, flush=True)

        if args.dry_run:
            command = build_recovery_command(args, trial_dir, sample)
            print("[supervisor] dry run command:", " ".join(command), file=log, flush=True)
            summary = {"outcome": "dry_run", "reason": "not_executed", **sample.to_dict()}
            write_json(trial_dir / "summary.json", summary)
            return summary

        prepared = prepare_drop(master, state, supervisor_config, sample.drop_altitude_m, args.takeoff_timeout, log)
        if not prepared and args.require_drop_altitude:
            summary = {"outcome": "setup_failed", "reason": "drop_altitude_not_reached", **sample.to_dict()}
            write_json(trial_dir / "summary.json", summary)
            return summary

        command = build_recovery_command(args, trial_dir, sample)
        print("[supervisor] starting recovery:", " ".join(command), file=log, flush=True)
        process = subprocess.Popen(
            command,
            cwd=args.repo_root,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(args.recovery_warmup_s)

        print("[supervisor] force-disarming to create drop", file=log, flush=True)
        command_force_disarm(master)

        try:
            return_code = process.wait(timeout=args.trial_timeout)
        except subprocess.TimeoutExpired:
            print("[supervisor] recovery process timed out; terminating", file=log, flush=True)
            process.terminate()
            return_code = None

        parsed = parse_latest_recovery_log(trial_dir, ground_altitude_m=args.ground_altitude)
        summary = {
            **sample.to_dict(),
            **asdict(parsed),
            "recovery_return_code": return_code,
        }
        write_json(trial_dir / "summary.json", summary)
        print(f"[supervisor] outcome={summary['outcome']} reason={summary['reason']}", file=log, flush=True)
        return summary


def append_results(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run randomized live PX4 SITL recovery trials.")
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--supervisor-connection", default="udpin:0.0.0.0:14561")
    parser.add_argument("--recovery-connection", default="udpin:0.0.0.0:14560")
    parser.add_argument("--min-drop-altitude", type=float, default=25.0)
    parser.add_argument("--max-drop-altitude", type=float, default=60.0)
    parser.add_argument("--takeoff-timeout", type=float, default=45.0)
    parser.add_argument("--trial-timeout", type=float, default=30.0)
    parser.add_argument("--recovery-warmup-s", type=float, default=1.0)
    parser.add_argument("--ground-altitude", type=float, default=1.0)
    parser.add_argument("--require-drop-altitude", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    session_dir = args.session_dir or args.repo_root / "logs" / f"monte_carlo_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    session_dir.mkdir(parents=True, exist_ok=True)

    seeds = trial_seeds(args.seed, args.trials)
    write_json(
        session_dir / "session_config.json",
        {
            "seed": args.seed,
            "trials": args.trials,
            "supervisor_connection": args.supervisor_connection,
            "recovery_connection": args.recovery_connection,
            "session_dir": str(session_dir),
            "dry_run": args.dry_run,
        },
    )

    master = None
    state = arm.VehicleState()
    supervisor_config = arm.Config(connection=args.supervisor_connection)
    if not args.dry_run:
        master = arm.open_connection(args.supervisor_connection, supervisor_config.connection_timeout_s)
        arm.request_basic_telemetry(master)

    rows: list[dict] = []
    for index, seed in enumerate(seeds, start=1):
        print(f"[monte-carlo] trial {index}/{args.trials} seed={seed}")
        row = run_trial(args, session_dir, master, state, supervisor_config, index, seed)
        rows.append(row)
        append_results(session_dir / "results.csv", rows)

    print(f"[monte-carlo] results: {session_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

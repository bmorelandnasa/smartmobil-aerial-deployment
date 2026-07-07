from __future__ import annotations

import argparse
import re
from pathlib import Path

from recovery_controller import VerticalControllerConfig, VerticalControllerInput, VerticalRecoveryController


SAMPLE_RE = re.compile(
    r"alt=(?P<alt>-?\d+(?:\.\d+)?)m .*?vz=(?P<vz>-?\d+(?:\.\d+)?)m/s"
    r"(?:.*?target=(?P<target>-?\d+(?:\.\d+)?)m)?"
)


def parse_samples(log_path: Path, sample_dt_s: float):
    now_s = 0.0
    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not any(prefix in line for prefix in ("[catch]", "[recover]", "[hold]")):
            continue
        match = SAMPLE_RE.search(line)
        if not match:
            continue
        yield VerticalControllerInput(
            now_s=now_s,
            altitude_m=float(match.group("alt")),
            downward_speed_mps=float(match.group("vz")),
            target_altitude_m=float(match.group("target")) if match.group("target") else None,
            velocity_fresh=True,
        )
        now_s += sample_dt_s


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay a simple_hover3 log through the vertical recovery controller.")
    parser.add_argument("log", type=Path, help="Path to a simple_hover3 log file.")
    parser.add_argument("--hover-thrust", type=float, default=VerticalControllerConfig.hover_thrust)
    parser.add_argument("--velocity-p-gain", type=float, default=VerticalControllerConfig.velocity_p_gain)
    parser.add_argument("--velocity-i-gain", type=float, default=VerticalControllerConfig.velocity_i_gain)
    parser.add_argument("--velocity-d-gain", type=float, default=VerticalControllerConfig.velocity_d_gain)
    parser.add_argument("--climb-brake-gain", type=float, default=VerticalControllerConfig.climb_brake_gain)
    parser.add_argument("--thrust-slew-rate", type=float, default=VerticalControllerConfig.thrust_slew_rate_per_s)
    parser.add_argument("--max-descent-speed", type=float, default=VerticalControllerConfig.max_descent_speed_mps)
    parser.add_argument("--controlled-max-thrust", type=float, default=VerticalControllerConfig.controlled_max_thrust)
    parser.add_argument("--climb-brake-min-thrust", type=float, default=VerticalControllerConfig.climb_brake_min_thrust)
    parser.add_argument("--sample-dt", type=float, default=0.5, help="Seconds between status rows in the log.")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    config = VerticalControllerConfig(
        hover_thrust=args.hover_thrust,
        velocity_p_gain=args.velocity_p_gain,
        velocity_i_gain=args.velocity_i_gain,
        velocity_d_gain=args.velocity_d_gain,
        climb_brake_gain=args.climb_brake_gain,
        thrust_slew_rate_per_s=args.thrust_slew_rate,
        max_descent_speed_mps=args.max_descent_speed,
        controlled_max_thrust=args.controlled_max_thrust,
        climb_brake_min_thrust=args.climb_brake_min_thrust,
    )
    controller = VerticalRecoveryController(config)
    for sample in parse_samples(args.log, args.sample_dt):
        output = controller.update(sample)
        print(
            f"t={sample.now_s:6.2f}s "
            f"alt={sample.altitude_m:7.1f}m "
            f"vz={sample.downward_speed_mps:7.1f}m/s "
            f"thrust={output.thrust:.2f} "
            f"v_sp={output.desired_down_speed_mps:5.1f} "
            f"{output.reason}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

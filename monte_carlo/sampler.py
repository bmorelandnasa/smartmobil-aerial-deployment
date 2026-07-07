from __future__ import annotations

import random
from dataclasses import asdict, dataclass


@dataclass
class TrialSample:
    trial_index: int
    seed: int
    drop_altitude_m: float
    recovery_thrust: float
    pid_kp: float
    pid_ki: float
    pid_kd: float
    catch_duration_s: float
    tilt_moderate_deg: float
    tilt_bad_deg: float
    tilt_inverted_deg: float
    handoff_after_offboard_s: float
    handoff_timeout_s: float
    initial_local_position_delay_s: float
    heartbeat_gap_s: float
    freefall_detection_delay_s: float
    history_window_s: float

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


@dataclass
class SamplerConfig:
    base_recovery_thrust: float = 0.55
    base_pid_kp: float = 0.045
    base_pid_ki: float = 0.0
    base_pid_kd: float = 0.025
    base_tilt_moderate_deg: float = 35.0
    base_tilt_bad_deg: float = 60.0
    base_tilt_inverted_deg: float = 90.0
    base_handoff_after_offboard_s: float = 0.5
    base_handoff_timeout_s: float = 2.0
    base_history_window_s: float = 2.0
    min_drop_altitude_m: float = 25.0
    max_drop_altitude_m: float = 60.0


def make_trial_sample(trial_index: int, seed: int, config: SamplerConfig) -> TrialSample:
    rng = random.Random(seed)

    def gain(value: float) -> float:
        if value == 0.0:
            return 0.0
        return value * rng.uniform(0.85, 1.15)

    return TrialSample(
        trial_index=trial_index,
        seed=seed,
        drop_altitude_m=rng.uniform(config.min_drop_altitude_m, config.max_drop_altitude_m),
        recovery_thrust=rng.uniform(config.base_recovery_thrust - 0.03, config.base_recovery_thrust + 0.03),
        pid_kp=gain(config.base_pid_kp),
        pid_ki=gain(config.base_pid_ki),
        pid_kd=gain(config.base_pid_kd),
        catch_duration_s=rng.uniform(0.15, 0.40),
        tilt_moderate_deg=config.base_tilt_moderate_deg + rng.uniform(-5.0, 5.0),
        tilt_bad_deg=config.base_tilt_bad_deg + rng.uniform(-5.0, 5.0),
        tilt_inverted_deg=config.base_tilt_inverted_deg + rng.uniform(-5.0, 5.0),
        handoff_after_offboard_s=max(0.0, config.base_handoff_after_offboard_s + rng.uniform(-0.2, 0.2)),
        handoff_timeout_s=max(0.5, config.base_handoff_timeout_s + rng.uniform(-0.2, 0.2)),
        initial_local_position_delay_s=rng.uniform(0.050, 0.300),
        heartbeat_gap_s=rng.uniform(0.0, 0.100),
        freefall_detection_delay_s=rng.uniform(0.0, 0.250),
        history_window_s=max(0.25, config.base_history_window_s + rng.uniform(-0.25, 0.25)),
    )


def trial_seeds(base_seed: int, count: int) -> list[int]:
    rng = random.Random(base_seed)
    return [rng.randrange(1, 2**31 - 1) for _ in range(count)]


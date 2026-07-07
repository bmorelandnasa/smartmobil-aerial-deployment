from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class VerticalControllerConfig:
    hover_thrust: float = 0.72
    min_thrust: float = 0.20
    max_thrust: float = 1.00
    catch_thrust: float = 1.00
    controlled_max_thrust: float = 0.92
    climb_brake_min_thrust: float = 0.28
    safety_floor_altitude_m: float = 8.0
    min_stop_distance_m: float = 6.0
    velocity_filter_tau_s: float = 0.08
    max_descent_speed_mps: float = 2.0
    max_brake_accel_mps2: float = 18.0
    max_comfort_accel_mps2: float = 10.0
    velocity_p_gain: float = 0.055
    velocity_i_gain: float = 0.018
    velocity_d_gain: float = 0.010
    velocity_integral_limit: float = 8.0
    climb_brake_gain: float = 0.11
    target_altitude_gain: float = 0.012
    thrust_slew_rate_per_s: float = 2.8


@dataclass
class VerticalControllerState:
    filtered_down_speed_mps: Optional[float] = None
    previous_filtered_down_speed_mps: Optional[float] = None
    previous_time_s: Optional[float] = None
    velocity_error_integral: float = 0.0
    thrust_cmd: float = 0.0
    desired_down_speed_mps: float = 0.0
    reason: str = "INIT"


@dataclass
class VerticalControllerInput:
    now_s: float
    altitude_m: Optional[float]
    downward_speed_mps: Optional[float]
    target_altitude_m: Optional[float]
    velocity_fresh: bool


@dataclass
class VerticalControllerOutput:
    thrust: float
    desired_down_speed_mps: float
    filtered_down_speed_mps: float
    reason: str
    stop_distance_m: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class VerticalRecoveryController:
    def __init__(self, config: VerticalControllerConfig):
        self.config = config
        self.state = VerticalControllerState()

    def reset(self) -> None:
        self.state = VerticalControllerState()

    def update(self, sample: VerticalControllerInput) -> VerticalControllerOutput:
        cfg = self.config
        state = self.state
        dt_s = self._dt(sample.now_s)

        if not sample.velocity_fresh:
            thrust = self._limit_thrust_slew(cfg.catch_thrust, dt_s)
            state.reason = "STALE_BRAKE"
            return self._output(thrust, 0.0, 0.0, self._stop_distance(sample.altitude_m))

        down_speed = self._filtered_speed(sample.downward_speed_mps or 0.0, dt_s)
        stop_distance_m = self._stop_distance(sample.altitude_m)
        desired_down_speed = self._safe_descent_speed(stop_distance_m)
        target_error = 0.0
        if sample.target_altitude_m is not None and sample.altitude_m is not None:
            target_error = sample.target_altitude_m - sample.altitude_m

        target_hold_allowed = (
            sample.target_altitude_m is not None
            and sample.altitude_m is not None
            and sample.altitude_m > cfg.safety_floor_altitude_m * 1.5
            and abs(down_speed) < 1.2
        )

        if target_hold_allowed:
            desired_down_speed = clamp(-target_error * 0.25, -0.8, 0.8)
        elif sample.altitude_m is not None and sample.altitude_m <= cfg.safety_floor_altitude_m * 1.5:
            desired_down_speed = 0.0

        velocity_error = down_speed - desired_down_speed
        if cfg.min_thrust < state.thrust_cmd < cfg.max_thrust or abs(velocity_error) < 1.5:
            state.velocity_error_integral = clamp(
                state.velocity_error_integral + velocity_error * dt_s,
                -cfg.velocity_integral_limit,
                cfg.velocity_integral_limit,
            )

        accel_down = 0.0
        if state.previous_filtered_down_speed_mps is not None:
            accel_down = (down_speed - state.previous_filtered_down_speed_mps) / max(dt_s, 0.001)

        thrust = cfg.hover_thrust
        thrust += velocity_error * cfg.velocity_p_gain
        thrust += state.velocity_error_integral * cfg.velocity_i_gain
        thrust += accel_down * cfg.velocity_d_gain

        if target_hold_allowed:
            thrust += clamp(target_error * cfg.target_altitude_gain, -0.10, 0.12)
        if down_speed < -0.5:
            climb_brake_thrust = cfg.hover_thrust - abs(down_speed) * cfg.climb_brake_gain
            thrust = min(thrust, climb_brake_thrust)

        required_up_accel = max(0.0, (down_speed * down_speed) / (2.0 * stop_distance_m))
        panic = down_speed > desired_down_speed + 6.0 or required_up_accel > cfg.max_brake_accel_mps2
        if panic:
            thrust = max(thrust, cfg.catch_thrust)
            state.reason = f"PANIC_STOP d={stop_distance_m:.1f}m v_sp={desired_down_speed:.1f}"
        elif down_speed > desired_down_speed + 0.5:
            thrust = min(thrust, cfg.controlled_max_thrust)
            state.reason = f"STOPPING d={stop_distance_m:.1f}m v_sp={desired_down_speed:.1f}"
        elif down_speed < -0.5:
            thrust = max(thrust, cfg.climb_brake_min_thrust)
            state.reason = f"BRAKE_CLIMB v_sp={desired_down_speed:.1f}"
        elif sample.target_altitude_m is not None:
            state.reason = "PID_HOLD"
        else:
            state.reason = "PID_RATE"

        state.desired_down_speed_mps = desired_down_speed
        thrust = clamp(thrust, cfg.min_thrust, cfg.max_thrust)
        thrust = self._limit_thrust_slew(thrust, dt_s)
        return self._output(thrust, desired_down_speed, down_speed, stop_distance_m)

    def _dt(self, now_s: float) -> float:
        if self.state.previous_time_s is None:
            dt_s = 0.01
        else:
            dt_s = clamp(now_s - self.state.previous_time_s, 0.001, 0.1)
        self.state.previous_time_s = now_s
        return dt_s

    def _filtered_speed(self, raw_down_speed: float, dt_s: float) -> float:
        cfg = self.config
        state = self.state
        if state.filtered_down_speed_mps is None:
            state.filtered_down_speed_mps = raw_down_speed
        alpha = dt_s / (cfg.velocity_filter_tau_s + dt_s)
        state.previous_filtered_down_speed_mps = state.filtered_down_speed_mps
        state.filtered_down_speed_mps += alpha * (raw_down_speed - state.filtered_down_speed_mps)
        return state.filtered_down_speed_mps

    def _stop_distance(self, altitude_m: Optional[float]) -> float:
        cfg = self.config
        if altitude_m is None:
            return cfg.min_stop_distance_m
        return max(cfg.min_stop_distance_m, altitude_m - cfg.safety_floor_altitude_m)

    def _safe_descent_speed(self, stop_distance_m: float) -> float:
        braking_limited_speed = (2.0 * self.config.max_comfort_accel_mps2 * stop_distance_m) ** 0.5
        return min(self.config.max_descent_speed_mps, braking_limited_speed)

    def _limit_thrust_slew(self, thrust: float, dt_s: float) -> float:
        state = self.state
        if state.thrust_cmd <= 0.0:
            state.thrust_cmd = thrust
            return thrust
        max_delta = self.config.thrust_slew_rate_per_s * dt_s
        state.thrust_cmd = clamp(thrust, state.thrust_cmd - max_delta, state.thrust_cmd + max_delta)
        return state.thrust_cmd

    def _output(
        self,
        thrust: float,
        desired_down_speed: float,
        filtered_down_speed: float,
        stop_distance_m: float,
    ) -> VerticalControllerOutput:
        state = self.state
        state.thrust_cmd = thrust
        state.desired_down_speed_mps = desired_down_speed
        return VerticalControllerOutput(
            thrust=thrust,
            desired_down_speed_mps=desired_down_speed,
            filtered_down_speed_mps=filtered_down_speed,
            reason=state.reason,
            stop_distance_m=stop_distance_m,
        )

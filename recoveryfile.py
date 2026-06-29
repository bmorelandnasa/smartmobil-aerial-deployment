from __future__ import annotations

import argparse
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Optional, Sequence

try:
    from pymavlink import mavutil as _pymavutil
except ImportError:
    _pymavutil = None


class _FallbackMavlink:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_DO_SET_MODE = 176
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_RESULT_ACCEPTED = 0
    MAV_RESULT_IN_PROGRESS = 5
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE = 1 << 7
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


MAVLINK = _pymavutil.mavlink if _pymavutil is not None else _FallbackMavlink()


@dataclass(frozen=True)
class Config:
    connection_string: str = "udpin:127.0.0.1:14560"
    stream_rate_hz: float = 20.0
    prime_stream_duration_s: float = 1.0
    freefall_speed_threshold_mps: float = 5.0
    freefall_time_threshold_s: float = 0.35
    telemetry_timeout_s: float = 0.5
    history_window_s: float = 2.0
    recovery_thrust: float = 0.65
    recovery_roll_rate_rad_s: float = 0.0
    recovery_pitch_rate_rad_s: float = 0.0
    recovery_yaw_rate_rad_s: float = 0.0
    ack_timeout_s: float = 2.0
    offboard_timeout_s: float = 3.0
    mode_request_interval_s: float = 0.5
    max_recovery_duration_s: Optional[float] = 10.0
    heartbeat_timeout_s: float = 2.0
    status_print_interval_s: float = 0.5


@dataclass(frozen=True)
class VelocityHistoryPoint:
    monotonic_time_s: float
    downward_speed_mps: float
    ground_speed_mps: float


@dataclass(frozen=True)
class TriggerContext:
    monotonic_time_s: float
    armed: bool
    flight_mode: Optional[str]
    downward_speed_mps: Optional[float]
    ground_speed_mps: Optional[float]
    velocity_is_fresh: bool
    velocity_timestamp_s: Optional[float]
    velocity_source: Optional[str]
    velocity_history: Sequence[VelocityHistoryPoint]


TriggerFn = Callable[[TriggerContext], bool]


def _format_optional(value: Optional[float], precision: int = 2) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{precision}f}"


@dataclass
class TelemetryCache:
    history_window_s: float
    armed: bool = False
    flight_mode: Optional[str] = None
    heartbeat_time_s: Optional[float] = None
    local_downward_speed_mps: Optional[float] = None
    local_ground_speed_mps: Optional[float] = None
    local_velocity_time_s: Optional[float] = None
    global_downward_speed_mps: Optional[float] = None
    global_ground_speed_mps: Optional[float] = None
    global_velocity_time_s: Optional[float] = None
    velocity_history: Deque[VelocityHistoryPoint] = field(default_factory=deque)

    def poll(self, master, timeout_s: float = 0.0) -> None:
        message = master.recv_match(blocking=timeout_s > 0.0, timeout=timeout_s)
        if message is not None:
            self.process_message(message, time.monotonic(), master=master)

        while True:
            message = master.recv_match(blocking=False, timeout=0)
            if message is None:
                return
            self.process_message(message, time.monotonic(), master=master)

    def process_message(self, message, now_s: float, master=None) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return

        if message_type == "HEARTBEAT":
            self.heartbeat_time_s = now_s
            base_mode = getattr(message, "base_mode", 0)
            self.armed = bool(base_mode & MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED)
            if master is not None and getattr(master, "flightmode", None):
                self.flight_mode = str(master.flightmode)
            elif hasattr(message, "custom_mode"):
                self.flight_mode = self.flight_mode or str(message.custom_mode)
            return

        if message_type == "LOCAL_POSITION_NED":
            downward_speed_mps = float(getattr(message, "vz", 0.0))
            vx = float(getattr(message, "vx", 0.0))
            vy = float(getattr(message, "vy", 0.0))
            ground_speed_mps = (vx * vx + vy * vy) ** 0.5
            self.local_downward_speed_mps = downward_speed_mps
            self.local_ground_speed_mps = ground_speed_mps
            self.local_velocity_time_s = now_s
            self._append_velocity_point(now_s, downward_speed_mps, ground_speed_mps)
            return

        if message_type == "GLOBAL_POSITION_INT":
            downward_speed_mps = float(getattr(message, "vz", 0.0)) / 100.0
            ground_speed_mps = float(getattr(message, "vel", 0.0)) / 100.0
            self.global_downward_speed_mps = downward_speed_mps
            self.global_ground_speed_mps = ground_speed_mps
            self.global_velocity_time_s = now_s
            self._append_velocity_point(now_s, downward_speed_mps, ground_speed_mps)

    def build_trigger_context(self, now_s: float, telemetry_timeout_s: float) -> TriggerContext:
        downward_speed_mps = None
        ground_speed_mps = None
        velocity_timestamp_s = None
        velocity_source = None

        if self.local_velocity_time_s is not None and now_s - self.local_velocity_time_s <= telemetry_timeout_s:
            downward_speed_mps = self.local_downward_speed_mps
            ground_speed_mps = self.local_ground_speed_mps
            velocity_timestamp_s = self.local_velocity_time_s
            velocity_source = "LOCAL_POSITION_NED"
        elif self.global_velocity_time_s is not None and now_s - self.global_velocity_time_s <= telemetry_timeout_s:
            downward_speed_mps = self.global_downward_speed_mps
            ground_speed_mps = self.global_ground_speed_mps
            velocity_timestamp_s = self.global_velocity_time_s
            velocity_source = "GLOBAL_POSITION_INT"

        return TriggerContext(
            monotonic_time_s=now_s,
            armed=self.armed,
            flight_mode=self.flight_mode,
            downward_speed_mps=downward_speed_mps,
            ground_speed_mps=ground_speed_mps,
            velocity_is_fresh=velocity_source is not None,
            velocity_timestamp_s=velocity_timestamp_s,
            velocity_source=velocity_source,
            velocity_history=tuple(self.velocity_history),
        )

    def heartbeat_is_fresh(self, now_s: float, timeout_s: float) -> bool:
        return self.heartbeat_time_s is not None and now_s - self.heartbeat_time_s <= timeout_s

    def status_text(self, context: TriggerContext, heartbeat_timeout_s: float) -> str:
        heartbeat_state = "fresh" if self.heartbeat_is_fresh(context.monotonic_time_s, heartbeat_timeout_s) else "stale"
        mode_text = self.flight_mode or "UNKNOWN"
        return (
            "status: "
            f"heartbeat={heartbeat_state} "
            f"armed={self.armed} "
            f"mode={mode_text} "
            f"source={context.velocity_source or 'none'} "
            f"down_mps={_format_optional(context.downward_speed_mps)} "
            f"ground_mps={_format_optional(context.ground_speed_mps)} "
            f"trigger_ready={context.velocity_is_fresh}"
        )

    def _append_velocity_point(self, now_s: float, downward_speed_mps: float, ground_speed_mps: float) -> None:
        self.velocity_history.append(
            VelocityHistoryPoint(
                monotonic_time_s=now_s,
                downward_speed_mps=downward_speed_mps,
                ground_speed_mps=ground_speed_mps,
            )
        )
        cutoff_s = now_s - self.history_window_s
        while self.velocity_history and self.velocity_history[0].monotonic_time_s < cutoff_s:
            self.velocity_history.popleft()


def build_freefall_trigger(config: Config) -> TriggerFn:
    def freefall_trigger(context: TriggerContext) -> bool:
        if not context.velocity_is_fresh or context.downward_speed_mps is None:
            return False

        if context.downward_speed_mps < config.freefall_speed_threshold_mps:
            return False

        window_start_s = context.monotonic_time_s - config.freefall_time_threshold_s
        relevant_history = [
            point
            for point in context.velocity_history
            if point.monotonic_time_s <= context.monotonic_time_s
        ]
        if not relevant_history:
            return False

        anchor_samples = [
            point
            for point in relevant_history
            if point.monotonic_time_s <= window_start_s
        ]
        if not anchor_samples:
            return False

        trailing_samples = [
            point
            for point in relevant_history
            if point.monotonic_time_s >= window_start_s
        ]
        if not trailing_samples:
            return False

        return all(
            point.downward_speed_mps >= config.freefall_speed_threshold_mps
            for point in (anchor_samples[-1], *trailing_samples)
        )

    return freefall_trigger


def default_trigger_name(trigger_fn: TriggerFn) -> str:
    return getattr(trigger_fn, "__name__", "custom-trigger")


def send_bodyrate_thrust_setpoint(master, config: Config, now_s: Optional[float] = None) -> None:
    now_s = time.monotonic() if now_s is None else now_s
    master.mav.set_attitude_target_send(
        int(now_s * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        MAVLINK.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
        [1.0, 0.0, 0.0, 0.0],
        config.recovery_roll_rate_rad_s,
        config.recovery_pitch_rate_rad_s,
        config.recovery_yaw_rate_rad_s,
        config.recovery_thrust,
    )


class SetpointStreamer:
    def __init__(self, master, config: Config) -> None:
        self.master = master
        self.config = config
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="setpoint-streamer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0 / max(self.config.stream_rate_hz, 1.0) + 0.5)

    def _run(self) -> None:
        interval_s = 1.0 / self.config.stream_rate_hz
        while not self._stop_event.is_set():
            send_bodyrate_thrust_setpoint(self.master, self.config)
            self._stop_event.wait(interval_s)


class RecoveryController:
    def __init__(self, master, config: Config, telemetry: TelemetryCache) -> None:
        self.master = master
        self.config = config
        self.telemetry = telemetry
        self.streamer = SetpointStreamer(master, config)
        self._sequence_lock = threading.Lock()

    def run_recovery_sequence(self, trigger_name: str) -> bool:
        with self._sequence_lock:
            print(f"trigger fired: {trigger_name}")
            self.streamer.start()
            print("recovery streaming active")

            try:
                self._poll_for(self.config.prime_stream_duration_s)

                arm_accepted = self._attempt_arm(force=False)
                if not arm_accepted:
                    print("arm denied")
                    print("force-arm attempted")
                    arm_accepted = self._attempt_arm(force=True)
                    print("force-arm accepted" if arm_accepted else "force-arm denied")
                else:
                    print("arm accepted")

                if not arm_accepted:
                    return False

                offboard_accepted = self._enter_offboard_mode()
                print("offboard accepted" if offboard_accepted else "offboard denied")
                if not offboard_accepted:
                    return False

                self._monitor_recovery()
                return True
            finally:
                self.streamer.stop()
                print("recovery streaming stopped")

    def _attempt_arm(self, force: bool) -> bool:
        force_param = 21196 if force else 0
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            force_param,
            0,
            0,
            0,
            0,
            0,
        )

        accepted = self._wait_for_command_ack(MAVLINK.MAV_CMD_COMPONENT_ARM_DISARM, self.config.ack_timeout_s)
        if accepted:
            return True

        return self.telemetry.armed

    def _enter_offboard_mode(self) -> bool:
        deadline_s = time.monotonic() + self.config.offboard_timeout_s
        last_request_s = 0.0

        while time.monotonic() < deadline_s:
            now_s = time.monotonic()
            if self._flight_mode_is_offboard():
                return True

            if now_s - last_request_s >= self.config.mode_request_interval_s:
                self._request_offboard_mode()
                last_request_s = now_s

            self.telemetry.poll(self.master, timeout_s=0.05)

        return self._flight_mode_is_offboard()

    def _request_offboard_mode(self) -> None:
        mapping = {}
        if hasattr(self.master, "mode_mapping"):
            try:
                mapping = self.master.mode_mapping() or {}
            except TypeError:
                mapping = {}

        offboard_mode = mapping.get("OFFBOARD") if mapping else None
        if offboard_mode is not None and hasattr(self.master, "set_mode"):
            self.master.set_mode(offboard_mode)
            return

        if hasattr(self.master.mav, "set_mode_send"):
            self.master.mav.set_mode_send(
                self.master.target_system,
                MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            )
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            MAVLINK.MAV_CMD_DO_SET_MODE,
            0,
            MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            0,
            0,
            0,
            0,
            0,
        )

    def _monitor_recovery(self) -> None:
        started_s = time.monotonic()
        while True:
            self.telemetry.poll(self.master, timeout_s=0.05)
            now_s = time.monotonic()

            if not self.telemetry.heartbeat_is_fresh(now_s, self.config.heartbeat_timeout_s):
                return

            if not self.telemetry.armed:
                return

            if self.config.max_recovery_duration_s is not None:
                if now_s - started_s >= self.config.max_recovery_duration_s:
                    return

    def _wait_for_command_ack(self, command_id: int, timeout_s: float) -> bool:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            remaining_s = max(0.0, deadline_s - time.monotonic())
            message = self.master.recv_match(blocking=True, timeout=min(0.1, remaining_s))
            if message is None:
                continue

            now_s = time.monotonic()
            self.telemetry.process_message(message, now_s, master=self.master)
            if message.get_type() != "COMMAND_ACK":
                continue

            if getattr(message, "command", None) != command_id:
                continue

            result = getattr(message, "result", None)
            print(f"command ack: command={command_id} result={result}")
            return result in (MAVLINK.MAV_RESULT_ACCEPTED, MAVLINK.MAV_RESULT_IN_PROGRESS)

        print(f"command ack timeout: command={command_id}")
        return False

    def _flight_mode_is_offboard(self) -> bool:
        return (self.telemetry.flight_mode or "").upper() == "OFFBOARD"

    def _poll_for(self, duration_s: float) -> None:
        deadline_s = time.monotonic() + duration_s
        while time.monotonic() < deadline_s:
            self.telemetry.poll(self.master, timeout_s=min(0.05, deadline_s - time.monotonic()))


def connect(connection_string: str):
    if _pymavutil is None:
        raise RuntimeError("pymavlink is not installed. Install it before running this script.")

    print(f"connecting: {connection_string}")
    master = _pymavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    return master


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PX4 SITL offboard freefall recovery test script.")
    parser.add_argument("--connection", default=Config.connection_string, help="MAVLink connection string.")
    parser.add_argument("--freefall-speed", type=float, default=Config.freefall_speed_threshold_mps, help="Downward speed threshold in m/s.")
    parser.add_argument("--freefall-time", type=float, default=Config.freefall_time_threshold_s, help="Sustained freefall time threshold in seconds.")
    parser.add_argument("--thrust", type=float, default=Config.recovery_thrust, help="Normalized thrust during recovery.")
    parser.add_argument("--status-interval", type=float, default=Config.status_print_interval_s, help="Status print interval in seconds.")
    parser.add_argument("--max-recovery-duration", type=float, default=Config.max_recovery_duration_s, help="Maximum recovery streaming duration in seconds.")
    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    return Config(
        connection_string=args.connection,
        freefall_speed_threshold_mps=args.freefall_speed,
        freefall_time_threshold_s=args.freefall_time,
        recovery_thrust=args.thrust,
        status_print_interval_s=args.status_interval,
        max_recovery_duration_s=args.max_recovery_duration,
    )


def main(config: Optional[Config] = None, trigger_fn: Optional[TriggerFn] = None) -> int:
    config = config or Config()
    trigger_fn = trigger_fn or build_freefall_trigger(config)

    master = connect(config.connection_string)
    telemetry = TelemetryCache(history_window_s=config.history_window_s)
    controller = RecoveryController(master, config, telemetry)

    print(f"connection established: {config.connection_string}")
    print(f"freefall trigger: threshold={config.freefall_speed_threshold_mps:.2f}m/s duration={config.freefall_time_threshold_s:.2f}s")
    last_status_print_s = 0.0
    trigger_name = default_trigger_name(trigger_fn)
    while True:
        telemetry.poll(master, timeout_s=0.1)
        now_s = time.monotonic()
        context = telemetry.build_trigger_context(now_s, config.telemetry_timeout_s)
        if now_s - last_status_print_s >= config.status_print_interval_s:
            print(telemetry.status_text(context, config.heartbeat_timeout_s))
            last_status_print_s = now_s
        if trigger_fn(context):
            return 0 if controller.run_recovery_sequence(trigger_name) else 1


def cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return main(config=config_from_args(args))


if __name__ == "__main__":
    raise SystemExit(cli())

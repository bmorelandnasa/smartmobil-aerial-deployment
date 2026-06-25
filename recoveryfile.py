from __future__ import annotations

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
    connection_string: str = "udp:127.0.0.1:14540"
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

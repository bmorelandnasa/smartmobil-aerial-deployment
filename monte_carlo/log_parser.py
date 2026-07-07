from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


ALT_RE = re.compile(r"alt=(?P<alt>-?\d+(?:\.\d+)?)m")
VZ_RE = re.compile(r"vz=(?P<vz>-?\d+(?:\.\d+)?)m/s")


@dataclass
class RecoverySummary:
    outcome: str
    reason: str
    min_altitude_m: Optional[float]
    max_downward_speed_mps: Optional[float]
    handoff_seen: bool
    drop_seen: bool
    log_path: Optional[str]


def parse_recovery_log(log_path: Path, ground_altitude_m: float = 1.0) -> RecoverySummary:
    min_altitude: Optional[float] = None
    max_vz: Optional[float] = None
    handoff_seen = False
    drop_seen = False
    ground_seen = False
    timeout_seen = False

    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "[drop]" in line:
            drop_seen = True
        if "HOLD command accepted" in line or "HOLD heartbeat confirmed" in line:
            handoff_seen = True
        if "recovery timeout" in line or line.startswith("[done]"):
            timeout_seen = True

        alt_match = ALT_RE.search(line)
        if alt_match:
            altitude = float(alt_match.group("alt"))
            min_altitude = altitude if min_altitude is None else min(min_altitude, altitude)
            if altitude <= ground_altitude_m:
                ground_seen = True

        vz_match = VZ_RE.search(line)
        if vz_match:
            vz = float(vz_match.group("vz"))
            max_vz = vz if max_vz is None else max(max_vz, vz)

    if handoff_seen:
        outcome = "success"
        reason = "hold_handoff_confirmed"
    elif ground_seen:
        outcome = "failure"
        reason = "ground_or_near_ground"
    elif timeout_seen:
        outcome = "timeout"
        reason = "recovery_timeout_without_handoff"
    else:
        outcome = "unknown"
        reason = "no_terminal_marker"

    return RecoverySummary(
        outcome=outcome,
        reason=reason,
        min_altitude_m=min_altitude,
        max_downward_speed_mps=max_vz,
        handoff_seen=handoff_seen,
        drop_seen=drop_seen,
        log_path=str(log_path),
    )


def parse_latest_recovery_log(trial_dir: Path, ground_altitude_m: float = 1.0) -> RecoverySummary:
    logs = sorted(trial_dir.glob("simple_hover3_*.log"))
    if not logs:
        return RecoverySummary(
            outcome="unknown",
            reason="missing_recovery_log",
            min_altitude_m=None,
            max_downward_speed_mps=None,
            handoff_seen=False,
            drop_seen=False,
            log_path=None,
        )
    return parse_recovery_log(logs[-1], ground_altitude_m=ground_altitude_m)


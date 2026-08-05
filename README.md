# PX4 Freefall Recovery

Companion-computer prototype for recovering a PX4 multicopter after an in-air
disarm. The current implementation is intended for PX4 SITL with Gazebo and the
`gz_x500` model.

> This code force-arms a vehicle and commands motor thrust. Test in simulation
> before using hardware, and keep a separate manual kill method available.

## How it works

1. `simple_hover3.py` listens to PX4 telemetry through MAVProxy.
2. A drop is detected when downward local velocity remains above the configured
   threshold.
3. The script force-arms, requests Offboard mode, and streams level-attitude
   thrust setpoints at 50 Hz.
4. A PID controller adjusts thrust from vertical velocity. Thrust is limited
   while the vehicle is tilted.
5. When velocity, altitude, tilt, and body-rate checks pass, the script requests
   PX4 Hold (`AUTO.LOITER`) and continues monitoring telemetry.

MAVLink `LOCAL_POSITION_NED.vz` is positive downward. The altitude printed by
the script is `-LOCAL_POSITION_NED.z`, relative to the estimator's local origin.

## Requirements

- Python 3.10 or newer
- PX4 Autopilot with Gazebo Harmonic support
- QGroundControl (optional, but recommended for observation)
- WSL or Linux for the included MAVProxy launcher

Create a virtual environment in this repository and install the dependencies:

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

## Run a SITL test

Use separate terminals for each process.

From the PX4 Autopilot directory, start SITL:

```bash
make px4_sitl gz_x500
```

From this repository, start MAVProxy:

```bash
bash mavproxy.sh
```

MAVProxy listens for PX4 on UDP `14540` and forwards telemetry to:

- `127.0.0.1:14560` for `simple_hover3.py`
- `127.0.0.1:14561` for the Monte Carlo supervisor
- Windows/host port `14550` for QGroundControl

Start the recovery script:

```bash
python3 -m simple_hover3
```

Allow the vehicle to reach the test altitude, then disarm it to create the
simulated drop. Stop the script with `Ctrl+C`. Timestamped logs are written to
`logs/`.

Use `python3 -m simple_hover3 --help` to see all available settings. A typical
tuning run looks like:

```bash
python3 -m simple_hover3 \
  --recovery-thrust 0.55 \
  --catch-thrust 0.90 \
  --pid-kp 0.045 \
  --pid-ki 0.0 \
  --pid-kd 0.025
```

Thrust values are normalized from `0.0` to `1.0` and must be retuned for each
airframe. The defaults were developed against `gz_x500`; they are not hardware
calibration values.

## Project structure

| Path | Purpose |
| --- | --- |
| `simple_hover3.py` | Active drop detection, Offboard recovery, PID thrust control, Hold handoff, and logging |
| `simple_arm3.py` | Shared MAVLink connection, telemetry, state, freefall, and force-arm functions; can also run as an arm-only test |
| `mavproxy.sh` | Routes SITL MAVLink traffic to the recovery script, QGroundControl, and test supervisor |
| `requirements.txt` | Python dependencies |
| `monte_carlo/` | Optional repeated SITL trial harness and result parser |
| `recovery_controller.py` | Earlier standalone vertical-controller experiment; not used by the active recovery script |
| `replay_recovery_controller.py` | Replays text logs through the earlier controller |
| `freefall_arm.nsh` | Optional PX4-shell freefall arm experiment |
| `arm_after_delay.nsh` | Optional PX4-shell timed arm experiment |

The minimum files for the current recovery test are `simple_hover3.py`,
`simple_arm3.py`, `mavproxy.sh`, and `requirements.txt`.

## Console and logs

The default console uses the panel view. Use one-line output when collecting or
comparing logs:

```bash
python3 -m simple_hover3 --console-style line
```

Use `--no-log` to disable the timestamped log file. The important phases are
`wait`, `drop`, `recover`, `handoff`, and `hold`.

## Monte Carlo tests

The optional test harness can generate trial configurations without connecting
to SITL:

```bash
python3 -m monte_carlo.sitl_driver --trials 3 --seed 42 --dry-run
```

See `monte_carlo/README.md` for live and headless batch instructions.

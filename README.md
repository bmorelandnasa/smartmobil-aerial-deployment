# smartmobil-aerial-deployment

Minimal PX4/Gazebo SITL scripts for testing freefall detection, force-arm, and
Offboard setpoint streaming.

## Active files

- `simple_arm3.py` - MAVLink connection, telemetry parsing, freefall detection,
  and force-arm helpers.
- `simple_hover3.py` - waits for freefall, streams level attitude/thrust
  setpoints, requests Offboard, retries force-arm, then asks PX4 Hold to take
  over once descent has slowed.
- `mavproxy.sh` - MAVProxy routing helper for WSL + QGC.

## Run

Install dependencies:

```bash
pip install -r requirements.txt
```

Start PX4 SITL and MAVProxy, then run:

```bash
python3 -m simple_hover3
```

Useful tuning flags:

```bash
python3 -m simple_hover3 --base-thrust 0.85 --pid-kp 0.05 --pid-ki 0.01 --recovery-duration 10
```

Logs are written to `logs/` unless `--no-log` is passed.

## Monte Carlo SITL batches

Monte Carlo tooling lives in `monte_carlo/`. With PX4 SITL and MAVProxy running,
try a dry run first:

```bash
python3 -m monte_carlo.sitl_driver --trials 3 --seed 42 --dry-run
```

For live trials, use:

```bash
python3 -m monte_carlo.sitl_driver --trials 5 --seed 42
```

Each batch writes a session folder under `logs/` with per-trial configs,
supervisor logs, recovery logs, summaries, and a combined `results.csv`.

The PID target is vertical speed near `0.0 m/s`. More downward velocity increases
thrust; as the fall slows or turns into a climb, thrust decreases.

By default, the script keeps streaming Offboard thrust first. It only requests
PX4 Hold (`AUTO.LOITER`) after recovery has streamed for `0.5s`, arm has been
accepted, altitude is above `10m`, and vertical speed has slowed into the
`-3.0` to `2.0 m/s` handoff window. Preexisting Hold mode before the drop never
counts as a successful handoff; the script must first stream recovery, pass the
velocity gate, and explicitly request Hold. Tune that handoff window with:

```bash
python3 -m simple_hover3 --handoff-after-offboard 0.5 --handoff-vz 2.0 --handoff-climb-limit -3.0
```

If Hold is not heartbeat-confirmed, the script keeps recovery active and logs
that it is still waiting/continuing. Use `--no-handoff-hold` to disable the
handoff test.

MAVProxy/QGC can show `LOITER` before the Python script sees the matching PX4
heartbeat. The script now logs local-position age, velocity age, and heartbeat
age separately, and it treats an accepted mode ACK after a Hold request as a
successful active-recovery handoff. Treat the `mode=...` field, ACK lines, and
MAVProxy's `Mode LOITER` line together when judging the handoff.

# Monte Carlo SITL harness

This folder contains the live PX4 SITL Monte Carlo harness for the recovery script.

## Routing

Run MAVProxy with two local outputs:

- `127.0.0.1:14560` for `simple_hover3.py`
- `127.0.0.1:14561` for the Monte Carlo supervisor

The repository `mavproxy.sh` includes both outputs.

## Dry run

```bash
python -m monte_carlo.sitl_driver --trials 3 --seed 42 --dry-run
```

This creates a session folder and sampled trial configs without connecting to SITL.

## Live run with an already-running stack

Start PX4 SITL and MAVProxy first, then run:

```bash
python -m monte_carlo.sitl_driver --trials 5 --seed 42
```

## Live run with stack launch

To have the Monte Carlo driver launch headless PX4 SITL and MAVProxy, pass the
PX4 source directory:

```bash
python -m monte_carlo.sitl_driver \
  --launch-stack \
  --px4-dir /path/to/PX4-Autopilot \
  --trials 5 \
  --seed 42
```

By default this launches:

```bash
make HEADLESS=1 px4_sitl gz_x500
bash mavproxy.sh
```

For each live trial, the supervisor streams an Offboard local-position target to
climb to the sampled drop altitude, starts `simple_hover3.py`, then force-disarms
to create the drop.

After the recovery script exits, the supervisor force-disarms again, waits for
local-position altitude to return near ground, and then starts the next trial in
the same SITL session.

`time_to_recovery_s` is measured from the drop command until local vertical speed
stays between `--stable-min-vz` and `--stable-max-vz` for `--stable-dwell`
seconds while local altitude remains above zero. Defaults are `-0.5 m/s`,
`+1.0 m/s`, and `1.0 s`.

Each session writes:

- `session_config.json`
- `results.csv`
- `trial_0001/trial_config.json`
- `trial_0001/supervisor.log`
- `trial_0001/simple_hover3_*.log`
- `trial_0001/summary.json`

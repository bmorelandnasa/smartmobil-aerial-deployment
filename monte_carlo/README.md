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

## Live run

Start PX4 SITL and MAVProxy first, then run:

```bash
python -m monte_carlo.sitl_driver --trials 5 --seed 42
```

For each live trial, the supervisor streams an Offboard local-position target to
climb to the sampled drop altitude, starts `simple_hover3.py`, then force-disarms
to create the drop.

Each session writes:

- `session_config.json`
- `results.csv`
- `trial_0001/trial_config.json`
- `trial_0001/supervisor.log`
- `trial_0001/simple_hover3_*.log`
- `trial_0001/summary.json`

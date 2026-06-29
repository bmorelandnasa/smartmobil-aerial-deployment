# smartmobil-aerial-deployment

Traceback (most recent call last):
  File "/usr/lib/python3.10/runpy.py", line 196, in _run_module_as_main
    return _run_code(code, main_globals, None,
  File "/usr/lib/python3.10/runpy.py", line 86, in _run_code
    exec(code, run_globals)
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 194, in <module>
    raise SystemExit(cli())
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 190, in cli
    return run(config_from_args(build_arg_parser().parse_args()))
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 148, in run
    if not enter_offboard(master, state, config, recovery_started_s):
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 106, in enter_offboard
    request_offboard_mode(master)
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 67, in request_offboard_mode
    float(arm.MAVLINK.PX4_CUSTOM_MAIN_MODE_OFFBOARD),
AttributeError: module 'pymavlink.dialects.v10.ardupilotmega' has no attribute 'PX4_CUSTOM_MAIN_MODE_OFFBOARD'

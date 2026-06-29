# smartmobil-aerial-deployment

Traceback (most recent call last):
  File "/usr/lib/python3.10/runpy.py", line 196, in _run_module_as_main
    return _run_code(code, main_globals, None,
  File "/usr/lib/python3.10/runpy.py", line 86, in _run_code
    exec(code, run_globals)
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 210, in <module>
    raise SystemExit(cli())
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 206, in cli
    return run(config_from_args(build_arg_parser().parse_args()))
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 164, in run
    if not enter_offboard(master, state, config, recovery_started_s):
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 122, in enter_offboard
    request_offboard_mode(master)
  File "/home/bhmorela/px4_telem_plot/simple_hover.py", line 66, in request_offboard_mode
    master.set_mode(offboard_mode)
  File "/home/bhmorela/px4_telem_plot/venv/lib/python3.10/site-packages/pymavlink/mavutil.py", line 713, in set_mode
    self.set_mode_px4(mode, custom_mode, custom_sub_mode)
  File "/home/bhmorela/px4_telem_plot/venv/lib/python3.10/site-packages/pymavlink/mavutil.py", line 706, in set_mode_px4
    self.mav.command_long_send(self.target_system, self.target_component,
  File "/home/bhmorela/px4_telem_plot/venv/lib/python3.10/site-packages/pymavlink/dialects/v20/ardupilotmega.py", line 25205, in command_long_send
    self.send(self.command_long_encode(target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7), force_mavlink1=force_mavlink1)
  File "/home/bhmorela/px4_telem_plot/venv/lib/python3.10/site-packages/pymavlink/dialects/v20/ardupilotmega.py", line 20839, in send
    buf = mavmsg.pack(self, force_mavlink1=force_mavlink1)
  File "/home/bhmorela/px4_telem_plot/venv/lib/python3.10/site-packages/pymavlink/dialects/v20/ardupilotmega.py", line 12434, in pack
    return self._pack(mav, self.crc_extra, self.unpacker.pack(self.param1, self.param2, self.param3, self.param4, self.param5, self.param6, self.param7, self.command, self.target_system, self.target_component, self.confirmation), force_mavlink1=force_mavlink1)
struct.error: required argument is not a float

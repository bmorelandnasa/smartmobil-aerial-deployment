# PX4 / NuttX NSH timed force-arm fallback.
#
# Put this file on the Pixhawk SD card at:
#   /fs/microsd/etc/extras/arm_after_delay.nsh
#
# Run from the PX4 shell:
#   sh /fs/microsd/etc/extras/arm_after_delay.nsh
#
# This does not detect freefall. It waits, then force-arms.

set START_DELAY_US 300000
set ARM_DELAY_US 100000

echo arm_after_delay: force arming soon

usleep ${START_DELAY_US}

commander arm -f
usleep ${ARM_DELAY_US}
commander arm -f
usleep ${ARM_DELAY_US}
commander arm -f

echo arm_after_delay: force arm commands sent

# PX4 / NuttX NSH freefall force-arm script.
#
# Put this file on the Pixhawk SD card at:
#   /fs/microsd/etc/extras/freefall_arm.nsh
#
# Run from the PX4 shell:
#   sh /fs/microsd/etc/extras/freefall_arm.nsh
#
# Stop while waiting:
#   Ctrl-C

set SAMPLE_FILE /fs/microsd/freefall_arm_sample.txt
set CHECK_DELAY_US 50000
set ARM_DELAY_US 100000

echo freefall_arm: waiting for vehicle_land_detected freefall

while true
do
	listener vehicle_land_detected 1 > ${SAMPLE_FILE}

	if grep "freefall: true" ${SAMPLE_FILE}
	then
		echo freefall_arm: freefall detected
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		exit
	fi

	if grep "freefall: True" ${SAMPLE_FILE}
	then
		echo freefall_arm: freefall detected
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		exit
	fi

	if grep "freefall: 1" ${SAMPLE_FILE}
	then
		echo freefall_arm: freefall detected
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		usleep ${ARM_DELAY_US}
		commander arm -f
		exit
	fi

	usleep ${CHECK_DELAY_US}
done

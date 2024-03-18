#!/bin/bash
# Audiophonics
# Shutdown detection script
# Script to set GPIO 4 High for 1sec

PATH=/usr/local/bin:/usr/bin:/bin:/sbin:/usr/sbin

#echo "Setting pin GPIO4 high for 1 sec"
#sudo gpio -g mode 04 out
#sudo gpio -g write 04 1
#/bin/sleep 1
#sudo gpio -g write 04 0

echo "Informing power control switch to shutdown"
sudo /home/pi/serial_shutdown.sh

echo "Stopping services"
sudo systemctl stop mpd

echo "Shutdown"
sudo systemctl poweroff

exit 0

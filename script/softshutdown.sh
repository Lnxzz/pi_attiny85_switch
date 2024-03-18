#!/bin/bash

echo "Informing power control switch to shutdown"
sudo /home/pi/serial_shutdown.sh

echo "Stopping services"
sudo systemctl stop mpd

echo "Shutdown"
sudo systemctl poweroff

exit 0

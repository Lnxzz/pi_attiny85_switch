#!/bin/bash

echo "Informing power control switch to shutdown"
sudo /home/pi/serial_reboot.sh

echo "Stopping services"
sudo systemctl stop mpd

echo "Reboot"
sudo reboot

exit 0

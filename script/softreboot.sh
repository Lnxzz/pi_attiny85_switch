
#!/bin/bash
# Audiophonics
# Shutdown detection script
# Script to set GPIO 4 High for 1sec

PATH=/usr/local/bin:/usr/bin:/bin:/sbin:/usr/sbin

# Script to set GPIO 4 High and Reboot
# Reboot blink will stop after Boot OK return
#echo "setting pin GPIO 4 high for 1 sec"
#sudo gpio -g mode 04 out
#sudo gpio -g write 04 1
#sleep 1

echo "Informing power control switch to shutdown"
sudo /home/pi/serial_reboot.sh

echo "Stopping services"
sudo systemctl stop mpd

echo "Reboot"
sudo reboot

exit 0

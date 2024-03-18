#!/bin/bash
# configure serial port
/usr/bin/stty -F /dev/ttyAMA0 9600 raw -echo

# read data from the serial port till we receive the shutdown command
SER_PROCEED=1
while [ $SER_PROCEED==1 ];
do
  read SER_INPUT < /dev/ttyAMA0
  echo $SER_INPUT
  if [[ $SER_INPUT =~ ^[[:space:]]*"shutdown".* ]]; then
    echo "received shutdown command via serial port"
    break
  fi
  sleep 1
done
echo "Initiating shutdown..."
sudo shutdown -h now

#!/bin/sh
# configure serial port
/usr/bin/stty -F /dev/ttyAMA0 9600 raw -echo

# send boot ok message and get response
exec 3</dev/ttyAMA0                     #redirect serial output to fd 3
  cat <&3 > /tmp/ttyAMA0R.dat &         #redirect serial output to file
  PID=$!                                #save pid to kill cat
    echo "REBOOT" > /dev/ttyAMA0        #send command string to the serial port
    sleep 0.2s                          #wait for response
  kill $PID                             #kill cat
  wait $PID 2>/dev/null                 #supress "Terminated" output

exec 3<&-                               #free fd 3
cat /tmp/ttyAMA0R.dat                   #dump captured data
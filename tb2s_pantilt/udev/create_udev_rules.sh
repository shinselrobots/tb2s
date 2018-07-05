#!/bin/bash

echo "remap the device serial port(ttyUSBX) to dynamixel (usb2ax device)"
echo "rplidar usb connection as /dev/dynamixel , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy usb2ax.rules to  /etc/udev/rules.d/"
echo "`rospack find tb2s_pantilt`/udev/usb2ax.rules"
sudo cp `rospack find tb2s_pantilt`/udev/usb2ax.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
ls -la /dev | grep "\->"
echo "finished. You might need to unplug and replug the device..."

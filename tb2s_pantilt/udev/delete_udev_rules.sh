#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  dynamixel (usb2ax device)"
echo "sudo rm   /etc/udev/rules.d/usb2ax.rules"
sudo rm   /etc/udev/rules.d/usb2ax.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"

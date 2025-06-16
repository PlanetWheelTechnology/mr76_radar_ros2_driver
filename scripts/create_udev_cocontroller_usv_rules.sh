#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  cocontroller_usv"
echo "adcp controller usb connection as /dev/cocontroller_usv , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy cocontroller_usv.rules to  /etc/udev/rules.d/"
sudo cp ./cocontroller_usv.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "

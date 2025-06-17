#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  serial2can"
echo "adcp controller usb connection as /dev/serial2can , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy serial2can.rules to  /etc/udev/rules.d/"
sudo cp ./serial2can.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "

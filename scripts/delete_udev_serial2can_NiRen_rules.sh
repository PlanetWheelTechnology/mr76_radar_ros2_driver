#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  cocontroller_usv"
echo "sudo rm   /etc/udev/rules.d/cocontroller_usv.rules"
sudo rm   /etc/udev/rules.d/cocontroller_usv.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish  delete"

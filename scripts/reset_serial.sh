#!/bin/bash

#$ cd /sys/bus/usb/devices/usb3/3-3
#$ sudo chmod o+w bConfigurationValue
#$ sudo echo 0 > bConfigurationValue

#To turn the hub back on:
#$ sudo echo 1 > bConfigurationValue

#// Turn off USB0
#sudo echo 0 > bConfigurationValue 

#// Turn on USB0
#sudo echo 1 > bConfigurationValue 


VAL=/sys/bus/usb/devices/usb3/3-3/bConfigurationValue
sudo chmod o+w $VAL
sudo echo 0 > $VAL

sudo echo 1 > $VAL

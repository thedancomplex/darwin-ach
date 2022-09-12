#!/bin/bash
PACKAGE_NAME=darwin-legacy
INSTALL_DIR=/etc/$PACKAGE_NAME

cd $INSTALL_DIR/ros2/build/darwin_lofaro_clock

while [ 1 -le 2 ]
do
	echo '-----------------------------------------'
	echo '----- Starting Darwin Lofaro Legacy -----'
	echo '----------------  Clock   ---------------'
	echo '----------------ROS 2 Node---------------'
	echo '-----------------------------------------'
	./darwin_lofaro_clock
	echo '!!! Failed - Restarting in 5 seconds !!!'
	sleep 5
done


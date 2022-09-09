#!/bin/bash
PACKAGE_NAME=darwin-legacy
INSTALL_DIR=/etc/$PACKAGE_NAME

cd $INSTALL_DIR/ros2/build/darwin_lofaro

while [ 1 -le 2 ]
do
	echo '-------------------------------------'
	echo '--- Starting Darwin Lofaro Legacy ---'
	echo '--------------ROS 2 Node-------------'
	echo '-------------------------------------'
	./darwin_lofaro
done


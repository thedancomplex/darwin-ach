# Darwin Lofaro Legacy
The Darwin Lofaro Legacy system is made to run on an origonal ROBOTIS Darwin OP platform utilizing ros2 as a middle ware to allow for communication between the origonal computer and a second computer such as a raspi.

# CM730 Driver

Designed to talk directly with the CM730.  Updates the state and reference

# Changes
* Call motors, FTs, and the CM730 to have a return status level of 1 (default is 2)
* Change the LEFT FT sensor to have an ID of 111.
* Change the RIGHT FT sensor to have an ID of 112.

# Todo:
* Impliment walking via ROS utilizing the ROBOTIS walking gate but comunicating via ROS
* Impliment alternative serial library to cut down on the latency and overhead caused by the dyn lib
* Make a version that just publishes the IMU data and takes upper body commands and a twist message for walking


# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from time import sleep

import rclpy

from std_msgs.msg import String

import time

# We do not recommend this style as ROS 2 provides timers for this purpose,
# and it is recommended that all nodes call a variation of spin.
# This example is only included for completeness because it is similar to examples in ROS 1.
# For periodic publication please see the other examples using timers.


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')

    publisher = node.create_publisher(String, '/darwin/clock', 10)

    msg = String()

    i = 0
    T = 0.01

    tick = time.time()
    tick2 = time.time()
    dt = 0.0;
    while rclpy.ok():
        tock = time.time()
        dt3 = tock - tick
        tick = tock
        dt = dt + dt3
        msg.data = 'd 19 -20.0 6 -20.0 5 -20.0'
        if ( i > 100 ):
          i = 0
          dt = dt/100.0
          f = 1/dt
          print(f)
          dt = 0.0
        i = i+1
          
        publisher.publish(msg)
        tock2 = time.time()
        dt2 = tock2 - tick2
        while( dt2 < T ):
          tock2 = time.time()
          dt2 = tock2 - tick2
          sleep(0.00001)  # seconds
        tick2 = tock2

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

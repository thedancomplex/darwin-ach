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

import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

g_node = None

x0 = 0.0;
y0 = 0.0;
z0 = 0.0;

total_msg = 0.0
err_msg   = 0.0
ex0 = 0.0
ey0 = 0.0
ez0 = 0.0
e_all = 0.0

def chatter_callback(msg):
    global x0, y0, z0
    global total_msg, err_msg
    global ex0, ey0, ez0
    global e_all
    eer = 0
    ex = 0
    ey = 0
    ez = 0
    if (x0 == msg.linear.x):
      eer = 1
      ex0 = ex0+1.0
      ex = 1
    if (y0 == msg.linear.y):
      eer = 1
      ey0 = ey0+1.0
      ey = 1
    if (z0 == msg.linear.z):
      eer = 1
      ez0 = ez0+1.0
      ez = 1

    if (eer > 0):
      err_msg = err_msg + 1.0

    if( (ex == 1) & (ey == 1) & (ez == 1) ):
      e_all = e_all + 1.0

    total_msg = total_msg + 1.0

    err_percent = err_msg / total_msg
    exp = ex0 / total_msg
    eyp = ey0 / total_msg
    ezp = ez0 / total_msg
    e_all_p = e_all / total_msg

    x0 = msg.linear.x
    y0 = msg.linear.y
    z0 = msg.linear.z

    tmp = msg.linear.x + msg.linear.y + msg.linear.z

    if (tmp > 100.0):
      print('------------------------------------------------------------')

    print(eer,end='')
    print('\t',end='')
    print(err_percent,end='')
    print(' ', end='')
    print(e_all_p,end='')
    print('\t',end='')
    print(exp,end='')
    print(' ',end='')
    print(eyp,end='')
    print(' ',end='')
    print(ezp,end='')
    print('\t',end='')
    print(msg.linear.x, end='')
    print(' ',end='')
    print(msg.linear.y, end='')
    print(' ',end='')
    print(msg.linear.z, end='')
    print();

def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('imu_test_sub')

    subscription = g_node.create_subscription(Twist, '/darwin/imu', chatter_callback, 10)
    subscription  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

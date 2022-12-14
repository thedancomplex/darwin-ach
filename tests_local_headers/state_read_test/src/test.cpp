/*******************************************************************************
* Copyright 2022 Daniel M. Lofaro
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Daniel M. Lofaro */

#include "lofaro_darwin.h"

int main()
{
  // Open
  darwin::setup("/dev/ttyUSB0");
  //darwin::setup("/dev/ttyDARWIN1");


  // Turn on motor power
  darwin::on();
 
  // Wait 1 second for power to turn on
  darwin::sleep(1.0);

  // Try to ping the Dynamixel
  // Get Dynamixel model number


  double tick = darwin::time();
  double tock = darwin::time();
  double tick2 = darwin::time();

  while(1)
  {
    // read 1 byte from address 5
    //lofaro::do_read(200, 3);
    darwin::get_imu_state_auto();
    darwin::get_ft_state_auto();
    darwin::get_motor_state_auto(1);
    darwin::sleep(0.002);
    
    bool do_loop = true;
    while(do_loop) 
    { 
      tock = darwin::time();
      double dt = tock - tick;
      if (dt > 0.001) do_loop = false;
      darwin::read_buffer(); 
      darwin::sleep(0.0001);
    }




    double tock2 = darwin::time();
    double dt2 = tock2 - tick2;
    while( dt2 < 0.01 )
    {
      darwin::sleep(0.00001);
      tock2 = darwin::time();
      dt2 = tock2 - tick2;
    }    
    tick2 = tock2;

//    printf("dt = %f\t f = %f\n", dt2, 1/dt2);
  //  darwin::print_state();
//    printf("----------------------------\n");
    printf("dt = %f, f= %f, %f, %f, %f\n",dt2, 1/dt2, darwin::imu_acc_x, darwin::imu_acc_y, darwin::imu_acc_z);
  }
  // Turn off motor power
  darwin::off();

  // Close port
  darwin::close();

  return 0;
}

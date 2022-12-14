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

// Enable using dyn serial instead of the one Dan Lofaro made
#define DYN_SERIAL 1

// The system
#include "lofaro_darwin.h"

int main()
{
  // Open
  darwin::setup("/dev/ttyUSB0");
  //darwin::setup("/dev/ttyDARWIN1");


  // Turn on motor power
  printf("Power = %d\n", darwin::on());
 
  // Wait 1 second for power to turn on
  darwin::sleep(1.0);

  // Try to ping the Dynamixel
  // Get Dynamixel model number

  double tick = darwin::time();
  double tock = darwin::time();
  int mot_i = 20;
  while(1)
  {
    // read 1 byte from address 5
    //lofaro::do_read(200, 3);
    darwin::get_imu_state_auto();
//    darwin::get_motor_state(mot_i);
    darwin::sleep(0.002);
    
    bool do_loop = true;
    while(do_loop) 
    { 
      tock = darwin::time();
      double dt = tock - tick;
      if (dt > 0.01) do_loop = false;
      darwin::read_buffer(); 
      darwin::sleep(0.0001);
    }

    printf("%f\t %f\t %f\t %f\n", 
           darwin::darwin_data.imu_state.voltage,
//           darwin::darwin_data.imu_state.gyro_x,
//           darwin::darwin_data.imu_state.gyro_y,
//           darwin::darwin_data.imu_state.gyro_z,
           darwin::darwin_data.imu_state.acc_x,
           darwin::darwin_data.imu_state.acc_y,
           darwin::darwin_data.imu_state.acc_z);


    tock = darwin::time();
//    darwin::print_state_motor(mot_i);
  }
  // Turn off motor power
  darwin::off();

  // Close port
  darwin::close();

  return 0;
}

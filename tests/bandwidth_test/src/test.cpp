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

#include "cm730.h"
#include <unistd.h>


void getFT()
{
    darwin::update_ft();
    printf("lx: %0.4f, ly: %0.4f, rx: %0.4f, ry: %0.4f, fsr_x: %0.4f, fsr_y: %0.4f, raised(l/r): %d %d\n",
            darwin::ft_left_x,
            darwin::ft_left_y,
            darwin::ft_right_y,
            darwin::ft_right_y,
            darwin::ft_fsr_x,
            darwin::ft_fsr_y,
            darwin::ft_fsr_raised_x,
            darwin::ft_fsr_raised_y);

}

void getIMU()
{
    darwin::update_imu();
    printf("gx: %0.4f,\t gy: %0.4f,\t gz: %0.4f,\t ax: %0.4f,\t ay: %0.4f,\t az: %0.4f,\t volt:%0.2f\n",
            darwin::imu_gyro_x,
            darwin::imu_gyro_y,
            darwin::imu_gyro_z,
            darwin::imu_acc_x,
            darwin::imu_acc_y,
            darwin::imu_acc_z,
            darwin::voltage);
}

int main()
{

  // Open
  darwin::open();


  // Turn on motor power
  darwin::on(ID_DARWIN);
 
  // Wait 1 second for power to turn on
  usleep(1000000);

  // Try to ping the Dynamixel
  // Get Dynamixel model number

  // read 1 byte from address 5
  uint8_t buff = darwin::read1byte(ID_CM730, CM730_ADDRESS_ID);

  printf("buff = %d\n",buff);
  
  for( int i = 0; i < 10; i++)
  {
    getFT();
    usleep(3000000);
  }

  // Turn off motor power
  darwin::off(ID_DARWIN);

  // Close port
  darwin::close();

  return 0;
}

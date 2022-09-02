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

#include "lofaro_serial.h"
#include <unistd.h>


int main()
{

  // Open
  lofaro::do_open();


  // Turn on motor power
  // darwin::on(ID_DARWIN);
 
  // Wait 1 second for power to turn on
  usleep(1000000);

  // Try to ping the Dynamixel
  // Get Dynamixel model number

  double tick = lofaro::get_time();
  double tock = lofaro::get_time();
  while(1)
  {
    // read 1 byte from address 5
    lofaro::do_read(200, 3);
    lofaro::do_read_buffer();
    usleep(10000);
    tock = lofaro::get_time();
    double dt = tock - tick;
    double f  = 1/dt;
    tick = tock;
    printf("dt = %f\t f = %2f\n",dt, f);
    
  }

  // Turn off motor power
  // darwin::off(ID_DARWIN);

  // Close port
  // darwin::close();

  return 0;
}

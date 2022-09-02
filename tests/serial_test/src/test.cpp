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
  darwin::open();


  // Turn on motor power
  // darwin::on(ID_DARWIN);
 
  // Wait 1 second for power to turn on
  darwin::sleep(1.0);

  // Try to ping the Dynamixel
  // Get Dynamixel model number

  double tick = darwin::time();
  double tock = darwin::time();
  while(1)
  {
    // read 1 byte from address 5
    //lofaro::do_read(200, 3);
    darwin::read(200, 38, 13);
    darwin::read_buffer();
    darwin::sleep(0.01); 
    tock = darwin::time();
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

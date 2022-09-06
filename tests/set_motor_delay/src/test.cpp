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
  darwin::open("/dev/ttyUSB0");


  // Turn on motor power
  darwin::on(ID_DARWIN);
 
  // Wait 1 second for power to turn on
  darwin::sleep(1.0);


  darwin::set_motor_delays();


  // wait 1 second for power off
  darwin::sleep(1.0);

  // Turn off motor power
  darwin::off(ID_DARWIN);

  // Close port
  darwin::close();

  return 0;
}

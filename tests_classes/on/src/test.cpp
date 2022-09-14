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

#define DARWIN_LOFARO_DYN 1
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

  // Close port
  darwin::close();

  return 0;
}

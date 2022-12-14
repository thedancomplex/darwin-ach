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

#include <sys/time.h>

double get_time()
{
  long seconds, useconds;
  double duration;
  timeval the_time;
  gettimeofday(&the_time, NULL);
  seconds = the_time.tv_sec;
  useconds = the_time.tv_usec;

  duration = seconds + useconds / 1000000.0;

//  printf("%f\n", duration);
  return duration;
}


int main()
{




  // Open
  darwin::open();


  // Turn on motor power
  darwin::on(ID_DARWIN);
 
  // Wait 1 second for power to turn on
  usleep(100000);

  double tick = get_time();
  double tock = get_time();
  uint8_t buff = 0;
  while(1) {

    tock = get_time();
    double T = tock - tick;
    double f = 1/T;
    printf("dt = %f\t f = %f,\t buff = %d\n",T,f, buff );
    tick = tock;
//    buff = darwin::read1byte(ID_CM730, CM730_ADDRESS_ID);
    buff = darwin::read1byte(200, CM730_ADDRESS_ID);
//    darwin::flush();
//    darwin::update_imu();
//    darwin::update_ft();
//    usleep(10000);
  }


  // Try to ping the Dynamixel
  // Get Dynamixel model number

  for (int i = 1; i <=20; i++)
  {
    darwin::ping(i);
  }
  darwin::ping(200);

  // Turn off motor power
  darwin::off(ID_DARWIN);

  // Close port
  darwin::close();

  return 0;
}

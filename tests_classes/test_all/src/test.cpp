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

  DarwinLofaro dl = DarwinLofaro();
  // Open
  dl.setup("/dev/ttyUSB0", true);
  //darwin::setup("/dev/ttyDARWIN1");


  // Turn on motor power
  printf("Power = %d\n", dl.on());
 
  // Wait 1 second for power to turn on
  dl.sleep(1.0);

  /* Set the sleep rate for 100hz*/
  dl.rate(100.0);

  double tick = dl.time();
  double tock = dl.time();

  double pos = 0.3;
  int    mot = 19;
  int pos_i = 0;

  while(1)
  {
    tock = dl.time();
    double dt = tock - tick;
    double f = 1.0/dt;
    tick = tock;
    
    printf("%f, %f\n", dt, f);

    pos_i++;

    if( pos_i > 200 )
    {
      pos_i = 0;
      pos = -pos;
    }

    dl.getImu();
    dl.getFt();

    for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
    {
      dl.setMotPos(i, 0.0);
      dl.setMotSpeed(i,  0.75);
      dl.setMotTorque(i, 0.5);
    }
    dl.stageMotor();
    dl.putMotor();

    dl.sleep();

  }

  // Close port
  dl.stop();

  return 0;
}

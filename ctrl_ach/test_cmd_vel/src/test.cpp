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

#include "lofaro_darwin_ach_client.h"
#include <unistd.h>
#include <string.h>

int main()
{ 
  /* Make System Object */
  DarwinAchClient dac = DarwinAchClient();
  double val = 0.01;


  while(true)
  {
    val = -val;
//    dac.stageCmdVelMode(WALKING_STOP);
    dac.stageCmdVelMode(WALKING_START);
    dac.stageCmdVelX(val);
//    dac.stageCmdVelY(val);
//    dac.stageCmdVelThetaZ(val);
    dac.postCmdVel();
    printf("Posted Val = %f m/s\n", val);
    dac.sleep(5.0);

    dac.stageCmdVelX(0.0);
    dac.stageCmdVelMode(WALKING_STOP);
    dac.postCmdVel();
    printf("Posted Val = %f m/s\n", 0.0);
    dac.sleep(5.0);
  }

  return 0;
}

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
#include "Walking.h"
#include <unistd.h>
#include <string.h>



#define INI_FILE_PATH       "Data/config.ini"

using namespace Robot;

int main()
{

  printf("load ini\n");
  minIni* ini = new minIni(INI_FILE_PATH);

  printf("Walking Instance\n");
  Walking::GetInstance()->LoadINISettings(ini);

  printf("Make system object\n");
  DarwinAchClient dac = DarwinAchClient();
  int r = 0;
  printf("Use cmd to turn on system\n");
  r = dac.cmd(DARWIN_CMD_ON, true);
  if( r == DARWIN_CMD_OK ) printf("Darwin Started\n");
  else printf("Darwin Fail to start\n");


  /* Motion Timer */
  printf("Start Motion Timer\n");
  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  motion_timer->Start();


  /* Start Walking */
  printf("Start Walking for 10 seconds\n");
  Walking::GetInstance()->Start();


  double tick  = dac.time();  
  double tick2 = dac.time();
  double val = 0.3;
  int mot = 19;
  while(true)
  {
    dac.getState();

    dac.stageRefPos(mot, val);
    dac.postRef();

    printf("ax= %f, ay=%f, az=%f\n",
            dac.darwin_state.imu.acc_x,
            dac.darwin_state.imu.acc_y,
            dac.darwin_state.imu.acc_z);
    dac.sleep(0.01);

  
   /* Test print of JointData */
   for(int i = 0; i < 21; i++) printf("%f, ", output_deg[i] );
   printf("\n");


    double tock2 = dac.time();
    if( (tock2 - tick2) > 3.0 )
    {
      val = -val;
      tick2 = tock2;
    }


    double tock = dac.time();
    if( (tock - tick) > 10.0 ) break;
  }

  /* Stop Walking */
  printf("Stop Walking\n");
  Walking::GetInstance()->Stop();
  dac.sleep(2.0);

  r = dac.cmd(DARWIN_CMD_OFF, true);
  if( r == DARWIN_CMD_OK ) printf("Darwin Stopped\n");

  return 0;
}

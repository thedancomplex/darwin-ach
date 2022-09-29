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

double i2d(uint16_t val)
{
  double the_out = (double)((int32_t)val - 512) / 1023.0;
  return the_out;
}

uint16_t d2i(double val)
{
  int32_t a = (int32_t)(val * 1023.0) + 512;
  if( a < 0 )   a = 0;
  if( a > 1023) a = 1023;
  
  return (uint16_t)a;
}


int imuGyro2Dyn(double val)
{
  int shift = 512;
  int the_out = d2i(val / IMU_GYRO_SCALE ) - shift;
  return the_out;
}



int main()
{

  printf("load ini\n");
  minIni* ini = new minIni(INI_FILE_PATH);

  printf("Walking Instance\n");
  Walking::GetInstance()->LoadINISettings(ini);

  printf("Make system object\n");
  DarwinAchClient dac = DarwinAchClient();
  dac.setRefMode(MODE_REF);

  int r = 0;
  printf("Use cmd to turn on system\n");
  r = dac.cmd(DARWIN_CMD_ON, true);
  if( r == DARWIN_CMD_OK ) printf("Darwin Started\n");
  else printf("Darwin Fail to start\n");


  /* Start Walking */
  printf("Start Walking for 10 seconds\n");
  Walking::GetInstance()->Start();

  /* Get into home positon */
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    dac.stageRefPos(i, 0.0);
  }
  dac.postRef();
  dac.sleep(2.0);

  dac.setRefMode(DARWIN_CMD_MODE_REF_WALKING);

  /* Goto Walking Stance */
  printf("Going into walking stance\n");
  Walking::GetInstance()->Process();
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    dac.stageRefPos(i, Walking::GetInstance()->getRefRad(i));
  }
  dac.postRef();
  dac.sleep(5.0);
  printf("Place on ground\n");

  /* Set Lower Body to be high torque and fast */
  for(int i = DARWIN_MOTOR_MIN_LOWER; i <= DARWIN_MOTOR_MAX_LOWER; i++)
  {
    dac.stageRefVel(i, 100.0);
    dac.stageRefTorque(i, 100.0);
  }
  dac.postRef();
  dac.sleep(3.0);

  dac.rate(125.0);  

  double tick  = dac.time();  
  double tick2 = dac.time();
  double tock  = dac.time();
  double tock2 = dac.time();

  while(true)
  {

//    Walking::GetInstance()->setStepADeg(-30.0);
//    Walking::GetInstance()->setStepXmm(30.0);
//    Walking::GetInstance()->setStepYmm(-30.0);

    dac.getState();

    Walking::GetInstance()->setGyroX( (double)imuGyro2Dyn(dac.darwin_state.imu.gyro_x) );
    Walking::GetInstance()->setGyroY( (double)imuGyro2Dyn(dac.darwin_state.imu.gyro_y) );

    Walking::GetInstance()->Process();
    for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
    {
      dac.stageRefPos( i, Walking::GetInstance()->getRefRad(i)        );
      dac.stagePGain(  i, (double)Walking::GetInstance()->getPGain(i) );
      dac.stageIGain(  i, (double)Walking::GetInstance()->getIGain(i) );
      dac.stageDGain(  i, (double)Walking::GetInstance()->getDGain(i) );
    }

    dac.postRef();

    printf("ax= %f, ay=%f, az=%f\n",
            dac.darwin_state.imu.acc_x,
            dac.darwin_state.imu.acc_y,
            dac.darwin_state.imu.acc_z);
    dac.sleep();

  
   /* Test print of JointData */
/*
   for(int i = 0; i < 21; i++) printf("%f, ", Walking::GetInstance()->getRefDeg(i) );
   printf("\n");
   for(int i = 0; i < 21; i++) printf("%f, ", Walking::GetInstance()->getRefRad(i) );
   printf("\n");
*/

    /* Stop Walking */
    tock = dac.time();
    if( (tock - tick) >  9.5 ) Walking::GetInstance()->Stop();
    if( (tock - tick) > 13.0 ) break;
  }

  r = dac.cmd(DARWIN_CMD_OFF, true);
  if( r == DARWIN_CMD_OK ) printf("Darwin Stopped\n");

  return 0;
}

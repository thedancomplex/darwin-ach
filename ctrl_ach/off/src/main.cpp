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

int main(int argc, char** argv)
{ 

  bool do_slow = false;
  
  /* Slow option set */
  if(argc > 1)
  {
    std::string str1(argv[1]);
    if   (str1.compare("slow") == 0) do_slow = true;
    else if(str1.compare("-h") ==0) 
    {
      printf("\n");
      printf("  Darwin Ach - Turn Off                                      \n");
      printf("  -----------------------------------------------------------\n");
      printf("  -h        : This menue                                     \n");
      printf("  slow      : Move to home position at default slow speed    \n");
      printf("              before turning off the motors                  \n");
      printf("  [no-args] : Turn off motors without goign to home posisiton\n");
      printf("\n");
      return 0;
    }
  }

  /* Make System Object */
  DarwinAchClient dac = DarwinAchClient();
  dac.setRefMode(MODE_REF);

  if(do_slow)
  {
    /* Get into home positon */
    for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
    {
      dac.stageRefVel(i, DARWIN_REF_VEL_0);
      dac.stageRefPos(i, DARWIN_REF_POS_0);
    }
    dac.postRef();
    dac.sleep(4.0);
  }

  int r = 0;

  /* Turn Off System */
  r = dac.cmd(DARWIN_CMD_OFF, true);
  if( r == DARWIN_CMD_OK ){ r=0; }
  else{ printf("1\n"); return 1; }

  dac.sleep(0.5);
  printf("0\n");
  return 0;
}

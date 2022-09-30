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
  double val = 0.3;


  while(true)
  {
    val = -val;
    dac.stageRefPos(5, val);
    dac.stageRefVel(5, 100.0);
    dac.stageRefPos(6, val);
    dac.stageRefVel(6, 100.0);
    dac.postRef();
    dac.sleep(1.0);
  }



  dac.postRef();
  dac.sleep(0.5);
  printf("0\n");
  return 0;
}

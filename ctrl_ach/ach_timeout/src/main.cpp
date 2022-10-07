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

#define LOOP_TIMEOUT 30
#define LOOP_TIMEOUT_SLEEP_PERIOD 0.02
#define LOOP_ENABLE_SECONDS 5.0
#define LOOP_ENABLE_SECONDS_COUNT LOOP_ENABLE_SECONDS / LOOP_TIMEOUT_SLEEP_PERIOD

int main(int argc, char *argv[])
{

  bool do_wait = true;
  if( argc >= 2)
  {
    std::string str = argv[1];

    if(str.compare("no_wait") == 0)
    {
      do_wait = false;
    }
  } 

  /* Make System Object */
  DarwinAchClient dac = DarwinAchClient();

  double tick_timeout = dac.time();

  int r = 0;

  while(true)
  {
    /* checking if there is a new time value */
    int d = dac.getTime();
    if ( d > 0 )
    {
      r++;
    }
    else
    {
      r--;
    }
    if( r < 0 ) r = 0;
  
    if( r > LOOP_ENABLE_SECONDS_COUNT ) break;

    dac.sleep(LOOP_TIMEOUT_SLEEP_PERIOD);
  }


  printf("1\n");
  return 1;
}

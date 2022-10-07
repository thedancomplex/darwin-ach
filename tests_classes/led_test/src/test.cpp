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
#include <lofaro_darwin.h>

int main()
{
  DarwinLofaro dl = DarwinLofaro();

  dl.setup(true);

  dl.rate(1.0);

  int si = 0;
  while(1)
  { 
    uint8_t b = 1;
    b = b << si;
    si++;
    if(si > 2) si = 0;

    dl.setLed(b);
    dl.sleep();
  }


  return 0;
}

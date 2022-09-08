#include <sys/time.h>



/*
// String Split
#include <iostream>
#include <vector>
#include <sstream>
#include <string>



void do_split(std::string const &str, const char delim,
    std::vector<std::string> &out)
{
 // construct a stream from the string
 std::stringstream ss(str);

 std::string s;
 while (std::getline(ss, s, delim)) {
     out.push_back(s);
 }
}

*/

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


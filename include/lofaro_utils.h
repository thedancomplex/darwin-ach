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


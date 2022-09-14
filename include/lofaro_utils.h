#define LOFARO_UTILS 1
#include <sys/time.h>
#include <unistd.h>
#include <cstddef>


class LofaroUtils
{
  public:
    LofaroUtils() {}

    double getTime()
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

    int sleep(double val)
    {
      long usec = (long)(val * 1000000);
      return usleep(usec);
    }

};


#define LOFARO_UTILS 1
#include <sys/time.h>
#include <unistd.h>
#include <cstddef>


class LofaroUtils
{
  public:
    LofaroUtils() 
    {
      this->tick = this->getTime();
      this->tock = this->getTime();
      this->T    = 1.0/this->hz;
    }

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

    int sleep()
    {
      tock = this->getTime();
      double dt = T - (tock - tick);
      if (dt < 0) dt = 0.0;
      this->sleep(dt);
      tick = this->getTime();
      return 0;
    }

    int rate(double hz_des)
    {
      if(hz_des <= 0) return 1;
      this->hz = hz_des;
      this->T  = 1.0/hz_des;
      return 0;
    }

  private:
    double tick = 0.0;
    double tock = 0.0;   
    double hz   = 100.0;
    double T    = 0.01;

};


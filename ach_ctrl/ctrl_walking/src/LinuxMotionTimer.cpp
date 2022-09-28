/*
 *   LinuxMotionTimer.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MotionModule.h"
#include "LinuxMotionTimer.h"

#include <stdlib.h>
#include <string.h>

using namespace Robot;

LinuxMotionTimer::LinuxMotionTimer(MotionManager* manager)
    : m_Manager(manager)
{
    this->m_FinishTimer = false;
    this->m_TimerRunning = false;
}

void *LinuxMotionTimer::TimerProc(void *param)
{
    LinuxMotionTimer *timer = (LinuxMotionTimer *)param;
    static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC,&next_time);

    while(!timer->m_FinishTimer)
    {
        next_time.tv_sec += (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) % 1000000000;

        if(timer->m_Manager != NULL)
            timer->m_Manager->Process();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    pthread_exit(NULL);
}

void LinuxMotionTimer::Start(void)
{

  printf("lmt1\n");
    int error;
    struct sched_param param;
  printf("lmt2\n");
    pthread_attr_t attr;

  printf("lmt3\n");
    pthread_attr_init(&attr);

  printf("lmt4\n");
    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  printf("lmt5\n");
    if(error != 0)
        printf("error = %d\n",error);
    error = pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
    if(error != 0)
        printf("error = %d\n",error);

  printf("lmt6\n");
    memset(&param, 0, sizeof(param));
  printf("lmt7\n");
    param.sched_priority = 31;// RT (Dan this is where the real-time stuff is)
  printf("lmt8\n");
    error = pthread_attr_setschedparam(&attr, &param);
  printf("lmt9\n");
    if(error != 0)
        printf("error = %d\n",error);

  printf("lmt10\n");
    // create and start the thread
    if((error = pthread_create(&this->m_Thread, &attr, this->TimerProc, this))!= 0)
    {
  printf("pthread error = %d\n",error);
    if(error == EAGAIN) printf("EAGAN\n");
    if(error == EINVAL) printf("EINVAL\n");
    if(error == EPERM) printf("EPERM\n");
        exit(-1);
    }

  printf("lmt11\n");
    this->m_TimerRunning=true;
  printf("lmt12\n");

}

void LinuxMotionTimer::Stop(void)
{
    int error=0;

    // seti the flag to end the thread
    if(this->m_TimerRunning)
    {
        this->m_FinishTimer = true;
        // wait for the thread to end
        if((error = pthread_join(this->m_Thread, NULL))!= 0)
            exit(-1);
        this->m_FinishTimer = false;
        this->m_TimerRunning = false;
    }
}

bool LinuxMotionTimer::IsRunning(void)
{
    return this->m_TimerRunning;
}

LinuxMotionTimer::~LinuxMotionTimer()
{
    this->Stop();
    this->m_Manager = NULL;
}

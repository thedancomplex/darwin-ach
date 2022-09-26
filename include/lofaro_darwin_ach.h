#define DARWIN_LOFARO_ACH 1
#define DARWIN_LOFARO_DYN 1

#include "lofaro_darwin.h"
#include "lofaro_includes_ach.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>



class DarwinAch
{
  public:
    DarwinAch();

  private:
    int main_loop();
    int main_loop(int mode_state);
    int main_loop(int mode_state, int mode_ref);
    int loop();
    int loop(double hz);
    int loop(double hz, int mode_state);
    int loop(double hz, int mode_state, int mode_ref);
    int do_ref(int mode);
    int do_state(int mode);
    DarwinLofaro* dl = new DarwinLofaro();

    /* Reference Channel */
    ach_channel_t chan_darwin_ref;  

    /* State Feedback Channel */
    ach_channel_t chan_darwin_state;

    /* Command channel */
    ach_channel_t chan_darwin_cmd;

    /* Command Channel Return */
    ach_channel_t chan_darwin_cmd_return;

    /* Data types */
    darwin_data_def_t darwin_ref;
    darwin_data_def_t darwin_state;
    darwin_cmd_def_t  darwin_cmd;
    darwin_cmd_def_t  darwin_cmd_return;

};

DarwinAch::DarwinAch()
{
  /* Zero Data */
  memset(&this->darwin_ref,          0, sizeof(this->darwin_ref));
  memset(&this->darwin_state,        0, sizeof(this->darwin_state));
  memset(&this->darwin_cmd,          0, sizeof(this->darwin_cmd));
  memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));

  /* Make Ach Channels */
  printf("ACH_OK = %d\n", ACH_OK);
  printf("ACH_EEXIST = %d\n", ACH_EEXIST);
  printf("ACH_ENOENT = %d\n", ACH_ENOENT);
  ach_status_t r = ach_open(&this->chan_darwin_ref,        DARWIN_ACH_CHAN_REF,        NULL);
  printf("r = %d\n", r);
               r = ach_open(&this->chan_darwin_state,      DARWIN_ACH_CHAN_STATE,      NULL);
  printf("r = %d\n", r);
               r = ach_open(&this->chan_darwin_cmd,        DARWIN_ACH_CHAN_CMD,        NULL);
  printf("r = %d\n", r);
               r = ach_open(&this->chan_darwin_cmd_return, DARWIN_ACH_CHAN_CMD_RETURN, NULL);
  printf("r = %d\n", r);

  /* Do initial put on the channel to make sure the exist */
  ach_put(&this->chan_darwin_ref,        &this->darwin_ref,        sizeof(this->darwin_ref));
  ach_put(&this->chan_darwin_state,      &this->darwin_state,      sizeof(this->darwin_state));
  ach_put(&this->chan_darwin_cmd,        &this->darwin_cmd,        sizeof(this->darwin_cmd));
  ach_put(&this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return));
}

int DarwinAch::loop()
{
  return this->loop(HZ_RATE_DEFAULT);
}

int DarwinAch::loop(double hz)
{
  return this->loop(hz, HZ_STATE_DEFAULT);
}

int DarwinAch::loop(double hz, int mode_state)
{
  return this->loop(hz, mode_state, HZ_REF_DEFAULT);
}

int DarwinAch::loop(double hz, int mode_state, int mode_ref)
{
  int ref = 0;

  this->dl->rate(hz);
  
  bool do_loop = true;
  while(do_loop)
  {
    this->main_loop(mode_state, mode_ref);
    this->dl->sleep();
  }

  if( ref > 0 ) ref = 1;
  return ref;
}


int ft_i = 0;
int upper_i = 0;
int DarwinAch::main_loop()
{
  return this->main_loop(HZ_STATE_DEFAULT);
}

int DarwinAch::main_loop(int mode_state)
{
  if (mode_state >= DARWIN_HZ_MODE_COUNT) return 1;
  return this->main_loop(mode_state, HZ_REF_DEFAULT);
}

int DarwinAch::main_loop(int mode_state, int mode_ref)
{

  size_t fs;
 
  /* Get the latest reference channel */
  ach_status_t r = ach_get( &this->chan_darwin_ref, &this->darwin_ref, sizeof(this->darwin_ref), &fs, NULL, ACH_O_LAST );
  if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}


  /* Get the latest cmd channel */
  r = ach_get( &this->chan_darwin_cmd, &this->darwin_cmd, sizeof(this->darwin_cmd), &fs, NULL, ACH_O_LAST );
  if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}


  int ret = 0;
  ret += this->do_ref(mode_ref);
  ret += this->do_state(mode_state);

  if( ret > 0 ) ret = 1;
  return ret;
}


int DarwinAch::do_ref(int mode)
{
  int ret = 0;
  /* Set Reference Here */
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
  {
    this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref.motor_ref[i].pos;
    this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref.motor_ref[i].speed;
    this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref.motor_ref[i].torque;
  }

  switch (mode)
  {
   case HZ_REF_SLOW_TOP:
   {
      const int LOWER_START = 7;
      const int LOWER_END   = 18;

      int upper_array[] = {1,2,3,4,5,6,19,20};
      int UPPER_LENGTH  = 8;

      /* Set one upper Ref per cycle */
      upper_i++;
      if(upper_i >= UPPER_LENGTH) upper_i = 0;
      ret += this->dl->stageMotor(upper_array[upper_i]);

      /* Always stage lower body */
      for(int i = LOWER_START; i <= LOWER_END; i++)
      {
        ret += this->dl->stageMotor(i);
      }
      break;
    }
    default: 
    {
      ret += this->dl->stageMotor();
      ret += this->dl->putMotor(); 
      break;
    }
  }
  if( ret > 1 ) ret = 1;
  return ret;
}


int DarwinAch::do_state(int mode)
{
  int ret = 0;
  /* Get State */
  switch (mode)
  {
    case HZ_STATE_50_IMU_MOTOR_FT:
    {
      ret += this->dl->getImu();
      ret += this->dl->getFt();
      ret += this->dl->getMotorSlow(1);
      break;
    }
    case HZ_STATE_50_IMU:
    {
      ret += this->dl->getImu();
      break;
    }
    case HZ_STATE_125_IMU:
    {
      ret += this->dl->getImu();
      break;
    }
    case HZ_STATE_100_IMU_FT_SLOW:
    {
      /* Get IMU State */
      ret += this->dl->getImu();

      /* Get Ft State every other cycle */
      if(ft_i == 0) ft_i = 1;
      else ft_i = 0;
      ret += this->dl->getFt(ft_i);
      break;
    }
    case HZ_STATE_100_IMU_MOTORS_SLOW:
    {
      /* Get IMU State */
      ret += this->dl->getImu();

      /* Get Motor State One per cycle */
      ret += this->dl->getMotorSlow(1);
    }
    default:
    {
      ret += 1;
      break;  
    }
  }

  this->darwin_state = this->dl->darwin_data;

  /* Put the data back on the Ach Channel */
  ach_put(&this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state));

  if( ret > 0 ) ret = 1;
  return ret;
}

#define DARWIN_GLOBALS 1

#ifndef DARWIN_DEFINES
#include "lofaro_defines.h"
#endif

darwin_data_def_t       darwin_data;
motor_state_def_t       motor_state[DARWIN_MOTOR_NUM+1];
motor_ref_def_t         motor_ref[DARWIN_MOTOR_NUM+1];
imu_state_def_t         imu_state;
ft_state_def_t          ft_state[DARWIN_FT_NUM];



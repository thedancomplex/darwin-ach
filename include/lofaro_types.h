#define DARWIN_TYPES 1	

#ifndef DARWIN_DEFINES
#include "lofaro_defines.h"
#endif

#include <stdint.h>

typedef struct imu_state_def {
        double acc_x;
        double acc_y;
        double acc_z;
        double gyro_x;
        double gyro_y;
        double gyro_z;
        double voltage;
}__attribute__((packed)) imu_state_def_t;

typedef struct ft_state_def {
        double s0;
        double s1;
        double s2;
        double s3;
        double x;
        double y;
        double voltage;
        uint8_t raised_x;
        uint8_t raised_y;
}__attribute__((packed)) ft_state_def_t;

typedef struct motor_state_def {
        double pos;
        double speed;
        double load;
        double voltage;
        double temp;
}__attribute__((packed)) motor_state_def_t;

typedef struct motor_ref_def {
        double pos;
        double speed;
        double load;
        double voltage;
}__attribute__((packed)) motor_ref_def_t;

typedef struct darwin_data_def {
  motor_ref_def_t     motor_ref[DARWIN_MOTOR_NUM+1];
  motor_state_def_t   motor_state[DARWIN_MOTOR_NUM+1];
  imu_state_def_t     imu;
  ft_state_def_t      ft[DARWIN_FT_NUM];
}__attribute__((packed)) darwin_data_def_t;


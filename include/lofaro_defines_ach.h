#define LOFARO_DEFINE_ACH 1

typedef enum {
        HZ_STATE_NULL,
        HZ_REF_NULL,
	HZ_STATE_50_IMU_MOTOR_FT,
	HZ_STATE_50_IMU,
	HZ_STATE_125_IMU,
	HZ_STATE_100_IMU_FT_SLOW,
	HZ_STATE_100_IMU_MOTORS_SLOW,
	HZ_REF_SLOW_TOP,
        DARWIN_HZ_MODE_COUNT
}__attribute__((packed)) darwin_hz_mode_index_t;

#define HZ_REF_DEFAULT HZ_REF_NULL
#define HZ_STATE_DEFAULT HZ_STATE_100_IMU_FT_SLOW
#define HZ_RATE_DEFAULT 100.0

#define DARWIN_ACH_CHAN_REF          "darwin-ach-chan-ref"
#define DARWIN_ACH_CHAN_STATE        "darwin-ach-chan-state"
#define DARWIN_ACH_CHAN_CMD          "darwin-ach-chan-cmd"
#define DARWIN_ACH_CHAN_CMD_RETURN   "darwin-ach-chan-ret"

//#include "lofaro_defines_ach.h"

#define DARWIN_REF_POS_0 0.0
#define DARWIN_REF_VEL_0 0.75
#define DARWIN_REF_TOR_0 0.5


typedef enum {
	DARWIN_CMD_OK,
	DARWIN_CMD_ON,
	DARWIN_CMD_OFF,
	DARWIN_CMD_START
        
}__attribute__((packed)) darwin_cmd_mode_index_t;

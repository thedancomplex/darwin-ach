#define LOFARO_DEFINES_ROS2           1

#define DARWIN_TOPIC_REF_POS         "/darwin/ref/position"
#define DARWIN_TOPIC_REF_VEL         "/darwin/ref/speed"
#define DARWIN_TOPIC_REF_TOR         "/darwin/ref/torque"
#define DARWIN_TOPIC_CMD             "/darwin/cmd"
#define DARWIN_TOPIC_CLOCK           "/darwin/clock"
#define DARWIN_TOPIC_STATE_IMU       "/darwin/state/imu"
#define DARWIN_TOPIC_STATE_FT_LEFT   "/darwin/state/ft/left"
#define DARWIN_TOPIC_STATE_FT_RIGHT  "/darwin/state/ft/right"
#define DARWIN_TOPIC_STATE_MOTOR_POS "/darwin/state/motor/position"
#define DARWIN_TOPIC_STATE_MOTOR_VEL "/darwin/state/motor/speed"
#define DARWIN_TOPIC_STATE_MOTOR_TOR "/darwin/state/motor/load"
#define DARWIN_TOPIC_STATE_MOTOR_VOL "/darwin/state/motor/voltage"
#define DARWIN_TOPIC_STATE_MOTOR_TMP "/darwin/state/motor/temperature"
#define DARWIN_TOPIC_STATE_TIME      "/darwin/time"
#define DARWIN_MOT_MIN                1
#define DARWIN_MOT_MAX                20

#define DARWIN_REF_POS_0              0.0
#define DARWIN_REF_VEL_0              0.75
#define DARWIN_REF_TOR_0              0.5
#define ENUM_FT_LEFT                  0
#define ENUM_FT_RIGHT                 1

#define RATE_100HZ                    0
#define RATE_125HZ                    1
#define RATE_50HZ                     2
#define RATE_100HZ_MOTOR_STATE        3
#define RATE_50HZ_IMU                 4

#define HZ_MODE_MOTORS                0
#define HZ_MODE_MOTORS_AND_STATE      1
#define HZ_IMU                        2
#define HZ_NULL                       3


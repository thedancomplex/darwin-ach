// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
int DXL_ID = 200;                   // Dynamixel ID: 200 - cm730
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#include <sys/ioctl.h>


// Enumbs and defines
#define CM730_ON  1
#define CM730_OFF 0
#define DARWIN_ON  CM730_ON
#define DARWIN_OFF CM730_OFF
#define IMU_ACC_SCALE 70.67723342939482
#define IMU_GYRO_SCALE 500.0
#define VOLTAGE_SCALE 10.0
#define FT_SCALE 1/1000.0
#define FSR_SCALE_X 1.0
#define FSR_SCALE_Y 1.0


// Motor IDs
#define ID_CM730 200
#define ID_DARWIN ID_CM730
#define ID_FT 100

// Addresses 
#define CM730_ADDRESS_DYN_POWER 24

#define CM730_ADDRESS_IMU_START 38
#define CM730_ADDRESS_IMU_LENGTH 12

#define CM730_ADDRESS_ID 3
#define CM730_ADDRESS_IMU_GYRO_Z 38
#define CM730_ADDRESS_IMU_GYRO_Y 40
#define CM730_ADDRESS_IMU_GYRO_X 42
#define CM730_ADDRESS_IMU_ACC_X 44
#define CM730_ADDRESS_IMU_ACC_Y 46
#define CM730_ADDRESS_IMU_ACC_Z 48

#define CM730_ADDRESS_VOLTAGE 50

#define FT_ADDRESS_START 26
#define FT_ADDRESS_LENGTH 10
#define FT_ADDRESS_LEFT_X 26
#define FT_ADDRESS_LEFT_Y 28
#define FT_ADDRESS_RIGHT_X 30
#define FT_ADDRESS_RIGHT_Y 32
#define FT_ADDRESS_FSR_X 34
#define FT_ADDRESS_FSR_Y 35
#define FT_ADDRESS_VOLTAGE 42


#define ERROR 1
#define NO_ERROR 0
#define RAISED 1
#define NOT_RAISED 0


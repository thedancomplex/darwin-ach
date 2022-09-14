# define DARWIN_DEFINE 1

# define M_PI                              3.14159265358979323846

// Buffer positions
#define BUFFER_ID                          2
#define BUFFER_LENGTH                      3       

// Enumbs and defines
#define CM730_ON                           1
#define DYN_ON                             CM730_ON  
#define CM730_OFF                          0
#define DYN_OFF                            CM730_OFF
#define DARWIN_ON                          CM730_ON
#define DARWIN_OFF                         CM730_OFF
//  #define IMU_ACC_SCALE 1.0
#define IMU_ACC_SCALE                      70.67723342939482
#define IMU_GYRO_SCALE                     500.0
#define VOLTAGE_SCALE                      10.0
#define FT_SCALE                           1/1000.0
#define FSR_SCALE_X                        1.0
#define FSR_SCALE_Y                        1.0
#define RETURN_OK                          0
#define RETURN_FAIL                        1
#define MOTOR_VOLTAGE_SCALE                0.1
#define MOTOR_POS_SCALE                    1.0 / 4096.0 * 2.0 * M_PI
#define MOTOR_SPEED_SCALE                  0.11 / 60.0  * 2.0 * M_PI
#define MOTOR_LOAD_SCALE                   1.0
#define MOTOR_TEMP_SCALE                   1.0
#define ENUM_FT_LEFT                       0
#define ENUM_FT_RIGHT                      1

#define DYN_ID_ALL                         0xfe

// Motor IDs
#define ID_CM730                           200
#define ID_DARWIN                          ID_CM730
#define ID_FT_LEFT                         112
#define ID_FT_RIGHT                        111
#define ID_FT                              1000


// Addresses 
#define CM730_ADDRESS_DYN_POWER            24
#define DYN_ADDRESS_DYN_POWER              CM730_ADDRESS_DYN_POWER
#define CM730_ADDRESS_ID                   3

#define CM730_ADDRESS_VOLTAGE              50
#define CM730_ADDRESS_STATUS_RETURN_LEVEL  16

#define CM730_ADDRESS_IMU_START            38
#define CM730_ADDRESS_IMU_LENGTH           12
#define CM730_ADDRESS_READ_DATA_OFFSET     5

#define CM730_ADDRESS_IMU_GYRO_Z           38
#define CM730_ADDRESS_IMU_GYRO_Y           40
#define CM730_ADDRESS_IMU_GYRO_X           42
#define CM730_ADDRESS_IMU_ACC_X            44
#define CM730_ADDRESS_IMU_ACC_Y            46
#define CM730_ADDRESS_IMU_ACC_Z            48

#define MX_ID                              2  
#define MX_ADDRESS_READ_DATA_OFFSET        5
#define MX_ADDRESS_STATE_START             36
#define MX_ADDRESS_STATE_LENGTH            8
#define MX_ADDRESS_POS                     36
#define MX_ADDRESS_SPEED                   38
#define MX_ADDRESS_LOAD                    40
#define MX_ADDRESS_VOLTAGE                 42
#define MX_ADDRESS_TEMP                    43
#define MX_ADDRESS_DELAY                   5
#define MX_ADDRESS_STATUS_RETURN_LEVEL     16
#define MX_ADDRESS_GOAL_POS                30

#define FT_ADDRESS_STATUS_RETURN_LEVEL     16
#define FT_ADDRESS_READ_DATA_OFFSET        5
#define FT_ADDRESS_START                   26
#define FT_ADDRESS_LENGTH                  10
#define FT_ADDRESS_S1                      26
#define FT_ADDRESS_S2                      28
#define FT_ADDRESS_S3                      30
#define FT_ADDRESS_S4                      32
#define FT_ADDRESS_FSR_X                   34
#define FT_ADDRESS_FSR_Y                   35
#define FT_ADDRESS_VOLTAGE                 42
#define FT_ADDRESS_DELAY                   5

#define SERIAL_PORT_DEFAULT                "/dev/ttyUSB0"

#define DARWIN_MOTOR_BROADCAST             0Xfe
#define DARWIN_MOTOR_NUM                   20
#define DARWIN_MOTOR_MIN                   1
#define DARWIN_MOTOR_MAX                   20

#define ERROR                              1
#define NO_ERROR                           0
#define RAISED                             1
#define NOT_RAISED                         0


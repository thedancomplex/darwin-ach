DARWIN_DEFINES = 1

M_PI                              = 3.14159265358979323846

# Buffer positions
BUFFER_ID                        =  2
BUFFER_LENGTH                    =  3       

# Enumbs and defines
CM730_ON                         =  1
DYN_ON                           =  CM730_ON  
CM730_OFF                        =  0
DYN_OFF                          =  CM730_OFF
DARWIN_ON                        =  CM730_ON
DARWIN_OFF                       =  CM730_OFF
#  IMU_ACC_SCALE 1.0
IMU_ACC_SCALE                    =  70.67723342939482
IMU_GYRO_SCALE                   =  500.0
VOLTAGE_SCALE                    =  10.0
FT_SCALE                         =  1/1000.0
FSR_SCALE_X                      =  1.0
FSR_SCALE_Y                      =  1.0
RETURN_OK                        =  0
RETURN_FAIL                      =  1
MOTOR_VOLTAGE_SCALE              =  0.1
MOTOR_POS_SCALE                  =  1.0 / 4096.0 * 2.0 * M_PI
MOTOR_SPEED_SCALE                =  0.11 / 60.0  * 2.0 * M_PI
MOTOR_LOAD_SCALE                 =  1.0
MOTOR_TEMP_SCALE                 =  1.0
MOTOR_ENC_REZ                    =  4096
MOTOR_SPEED_REZ                  =  2048
MOTOR_LOAD_REZ                   =  2048
ENUM_FT_LEFT                     =  0
ENUM_FT_RIGHT                    =  1
MOTOR_REF_SPEED_SCALE            =  0x3ff / (116.62 / 60.0 * 2.0 * M_PI)
# MOTOR_REF_SPEED_SCALE          =    0.114 / 60.0 * 2.0 * M_PI
# 0.114 Revolutions / Minute / tick * 1 Minute / 60.0 Seconds * 2.0 * M_PI Rad / Revolution
# 0.114 / 60.0 * 2.0 * M_PI Rad / Sec / tick

MOTOR_TORQUE_MAX                 =  1.0

DYN_ID_ALL                       =  0xfe
ID_ALL                           =  DYN_ID_ALL

# Motor IDs
ID_CM730                         =  200
ID_DARWIN                        =  ID_CM730
ID_FT_LEFT                       =  112
ID_FT_RIGHT                      =  111
ID_FT                            =  1000


# Addresses 
CM730_ADDRESS_DYN_POWER          =  24
DYN_ADDRESS_DYN_POWER            =  CM730_ADDRESS_DYN_POWER
CM730_ADDRESS_ID                 =  3

CM730_ADDRESS_VOLTAGE            =  50
CM730_ADDRESS_STATUS_RETURN_LEVEL=  16

CM730_ADDRESS_IMU_START          =  38
CM730_ADDRESS_IMU_LENGTH         =  12
CM730_ADDRESS_READ_DATA_OFFSET   =  5

CM730_ADDRESS_IMU_GYRO_Z         =  38
CM730_ADDRESS_IMU_GYRO_Y         =  40
CM730_ADDRESS_IMU_GYRO_X         =  42
CM730_ADDRESS_IMU_ACC_X          =  44
CM730_ADDRESS_IMU_ACC_Y          =  46
CM730_ADDRESS_IMU_ACC_Z          =  48
CM730_ADDRESS_LED_PANNEL         =  25


MX_ID                            =  2  
MX_ADDRESS_READ_DATA_OFFSET      =  5
MX_ADDRESS_STATE_START           =  36
MX_ADDRESS_STATE_LENGTH          =  8

MX_PACKET_PING                   =  0X01
MX_PACKET_READ                   =  0X02
MX_PACKET_WRITE                  =  0x03
MX_PACKET_REG_WRITE              =  0X04
MX_PACKET_ACTION                 =  0x05
MX_PACKET_FACTORY_RESET          =  0x06
MX_PACKET_REBOOT                 =  0x08
MX_PACKET_SYNC_WRITE             =  0x83
MX_PACKET_BULK_READ              =  0x92

MX_ADDRESS_POS_GOAL              =  30
MX_ADDRESS_VEL_GOAL              =  32
MX_ADDRESS_TORQUE_MAX            =  34
MX_ADDRESS_REF_LENGTH            =  6
MX_ADDRESS_REF_START             =  MX_ADDRESS_POS_GOAL

MX_ADDRESS_POS                   =  36
MX_ADDRESS_SPEED                 =  38
MX_ADDRESS_LOAD                  =  40
MX_ADDRESS_VOLTAGE               =  42
MX_ADDRESS_TEMP                  =  43
MX_ADDRESS_DELAY                 =  5
MX_ADDRESS_STATUS_RETURN_LEVEL   =  16
MX_ADDRESS_GOAL_POS              =  30
MX_ADDRESS_STATE_LENGTH          =  8
MX_ADDRESS_STATE_START           =  MX_ADDRESS_POS

MX_ADDRESS_P_GAIN                =  28
MX_ADDRESS_I_GAIN                =  27
MX_ADDRESS_D_GAIN                =  26


FT_ADDRESS_STATUS_RETURN_LEVEL   =  16
FT_ADDRESS_READ_DATA_OFFSET      =  5
FT_ADDRESS_START                 =  26
FT_ADDRESS_LENGTH                =  16
FT_ADDRESS_S1                    =  26
FT_ADDRESS_S2                    =  28
FT_ADDRESS_S3                    =  30
FT_ADDRESS_S4                    =  32
FT_ADDRESS_FSR_X                 =  34
FT_ADDRESS_FSR_Y                 =  35
FT_ADDRESS_VOLTAGE               =  42
FT_ADDRESS_DELAY                 =  5

SERIAL_PORT_DEFAULT              =  "/dev/ttyUSB0"
DEVICENAME                       =  SERIAL_PORT_DEFAULT


DARWIN_MOTOR_BROADCAST           =  0Xfe
DARWIN_MOTOR_NUM                 =  20
DARWIN_MOTOR_MIN                 =  1
DARWIN_MOTOR_MAX                 =  20
DARWIN_FT_NUM                    =  2
DARWIN_MOTOR_MIN_LOWER           =  7
DARWIN_MOTOR_MAX_LOWER           =  18
DARWIN_MOTOR_MIN_HEAD            =  19
DARWIN_MOTOR_MAX_HEAD            =  20
DARWIN_MOTOR_MIN_UPPER           =  1
DARWIN_MOTOR_MAX_UPPER           =  6


ERROR                            =  1
NO_ERROR                         =  0
RAISED                           =  1
NOT_RAISED                       =  0

BAUDRATE                         =  1000000
STDIN_FILENO                     =  0
SERIAL_PORT_LOW_LATENCY_DEFAULT  =  false

PROTOCOL_VERSION                 =  1.0

DARWIN_X                         =  1
DARWIN_Y                         =  2
DARWIN_Z                         =  3

DARWIN_ENUM_P_GAIN               =  1
DARWIN_ENUM_I_GAIN               =  2
DARWIN_ENUM_D_GAIN               =  3

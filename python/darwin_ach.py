class DarwinAch:
  def __init__(self):
    pass
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
  SERIAL_PORT_LOW_LATENCY_DEFAULT  =  False

  PROTOCOL_VERSION                 =  1.0

  DARWIN_X                         =  1
  DARWIN_Y                         =  2
  DARWIN_Z                         =  3

  DARWIN_ENUM_P_GAIN               =  1
  DARWIN_ENUM_I_GAIN               =  2
  DARWIN_ENUM_D_GAIN               =  3


  DARWIN_TOPIC_REF_POS         = "/darwin/ref/position"
  DARWIN_TOPIC_REF_VEL         = "/darwin/ref/speed"
  DARWIN_TOPIC_REF_TOR         = "/darwin/ref/torque"
  DARWIN_TOPIC_CMD             = "/darwin/cmd"
  DARWIN_TOPIC_CLOCK           = "/darwin/clock"
  DARWIN_TOPIC_STATE_IMU       = "/darwin/state/imu"
  DARWIN_TOPIC_STATE_FT_LEFT   = "/darwin/state/ft/left"
  DARWIN_TOPIC_STATE_FT_RIGHT  = "/darwin/state/ft/right"
  DARWIN_TOPIC_STATE_MOTOR_POS = "/darwin/state/motor/position"
  DARWIN_TOPIC_STATE_MOTOR_VEL = "/darwin/state/motor/speed"
  DARWIN_TOPIC_STATE_MOTOR_TOR = "/darwin/state/motor/load"
  DARWIN_TOPIC_STATE_MOTOR_VOL = "/darwin/state/motor/voltage"
  DARWIN_TOPIC_STATE_MOTOR_TMP = "/darwin/state/motor/temperature"
  DARWIN_TOPIC_STATE_TIME      = "/darwin/time"
  DARWIN_MOT_MIN               = 1
  DARWIN_MOT_MAX               = 20

  DARWIN_REF_POS_0             = 0.0
  DARWIN_REF_VEL_0             = 0.75
  DARWIN_REF_TOR_0             = 0.5
  ENUM_FT_LEFT                 = 0
  ENUM_FT_RIGHT                = 1

  RATE_100HZ                   = 0
  RATE_125HZ                   = 1
  RATE_50HZ                    = 2
  RATE_100HZ_MOTOR_STATE       = 3
  RATE_50HZ_IMU                = 4

  HZ_MODE_MOTORS               = 0
  HZ_MODE_MOTORS_AND_STATE     = 1
  HZ_IMU                       = 2
  HZ_NULL                      = 3


import threading
 
class thread(threading.Thread):
    def __init__(self, thread_name, r_info):
      threading.Thread.__init__(self)
      self.thread_name = thread_name
      self.ros_py = r_info[0]
      self.ros_node = r_info[1]
 
    # helper function to execute the threads
    def run(self):
      print(str(self.thread_name))
      self.ros_py.spin(self.ros_node)


class DarwinAchRos:
  from time import sleep
  import rclpy
  from std_msgs.msg import String
  from std_msgs.msg import Float64
  from std_msgs.msg import Float64MultiArray
  from geometry_msgs.msg import Twist
  PI = 3.14159265359

  def __init__(self, state=False):
    self.da = DarwinAch()
    self.rclpy.init()
    self.node = self.rclpy.create_node('darwin_simple_demo_ros2_python_publisher')
    self.pub_ref = self.node.create_publisher(self.String, self.da.DARWIN_TOPIC_REF_POS, 10)
    self.pub_vel = self.node.create_publisher(self.String, self.da.DARWIN_TOPIC_REF_VEL, 10)
    self.pub_tor = self.node.create_publisher(self.String, self.da.DARWIN_TOPIC_REF_TOR, 10)
    self.pub_cmd = self.node.create_publisher(self.String, self.da.DARWIN_TOPIC_CMD,     10)

    if state:
      self.sub_imu      = self.node.create_subscription(self.Twist,             self.da.DARWIN_TOPIC_STATE_IMU,       self.cb_state_imu,      10)
      self.sub_ft_left  = self.node.create_subscription(self.Twist,             self.da.DARWIN_TOPIC_STATE_FT_LEFT,   self.cb_state_ft_left,  10)
      self.sub_ft_right = self.node.create_subscription(self.Twist,             self.da.DARWIN_TOPIC_STATE_FT_RIGHT,  self.cb_state_ft_right, 10)
      self.sub_time     = self.node.create_subscription(self.Float64,           self.da.DARWIN_TOPIC_STATE_TIME,      self.cb_state_time,     10)
      self.sub_mot_pos  = self.node.create_subscription(self.Float64MultiArray, self.da.DARWIN_TOPIC_STATE_MOTOR_POS, self.cb_state_mot_pos,  10)
      self.sub_mot_vel  = self.node.create_subscription(self.Float64MultiArray, self.da.DARWIN_TOPIC_STATE_MOTOR_VEL, self.cb_state_mot_vel,  10)
      self.sub_mot_vol  = self.node.create_subscription(self.Float64MultiArray, self.da.DARWIN_TOPIC_STATE_MOTOR_VOL, self.cb_state_mot_vol,  10)
      self.sub_mot_tor  = self.node.create_subscription(self.Float64MultiArray, self.da.DARWIN_TOPIC_STATE_MOTOR_TOR, self.cb_state_mot_tor,  10)
      self.sub_mot_tmp  = self.node.create_subscription(self.Float64MultiArray, self.da.DARWIN_TOPIC_STATE_MOTOR_TMP, self.cb_state_mot_tmp,  10)
      self.thread_state = None
      self.imu_acc_x  = 0.0
      self.imu_acc_y  = 0.0
      self.imu_acc_z  = 0.0
      self.imu_gyro_x = 0.0
      self.imu_gyro_y = 0.0
      self.imu_gyro_z = 0.0
      self.time       = 0.0

      self.ft_left_x      = 0.0
      self.ft_left_y      = 0.0
      self.ft_left_lift   = False
      self.ft_left_lift_x = False
      self.ft_left_lift_y = False

      self.ft_right_x      = 0.0
      self.ft_right_y      = 0.0
      self.ft_right_lift   = False
      self.ft_right_lift_x = False
      self.ft_right_lift_y = False
      self.motor_position     = [None] * (self.da.DARWIN_MOTOR_NUM + 1)
      self.motor_velocity    = [None] * (self.da.DARWIN_MOTOR_NUM + 1)
      self.motor_torque      = [None] * (self.da.DARWIN_MOTOR_NUM + 1)
      self.motor_voltage     = [None] * (self.da.DARWIN_MOTOR_NUM + 1)
      self.motor_temperature = [None] * (self.da.DARWIN_MOTOR_NUM + 1)
      self.do_spin_thread()

    return
  
  def do_spin_thread(self):
    r_buff = (self.rclpy, self.node)
    self.thread_state = thread("StateThread",r_buff)
    self.thread_state.start()
    return

  def cb_state_imu(self, msg):
    self.imu_acc_x  = msg.linear.x
    self.imu_acc_y  = msg.linear.y
    self.imu_acc_z  = msg.linear.z
    self.imu_gyro_x = msg.angular.x
    self.imu_gyro_y = msg.angular.y
    self.imu_gyro_z = msg.angular.z
    return

  def cb_state_ft_left(self, msg):
    self.ft_left_x      = msg.linear.x
    self.ft_left_y      = msg.linear.y
    if( msg.linear.z > 0.1 ):
      self.ft_left_lift   = True
    else:
      self.ft_left_lift   = False

    if( msg.angular.x > 0.1 ):
      self.ft_left_lift_x   = True
    else:
      self.ft_left_lift_x   = False

    if( msg.angular.y > 0.1 ):
      self.ft_left_lift_y   = True
    else:
      self.ft_left_lift_y   = False
    
    return

  def cb_state_ft_right(self, msg):
    self.ft_right_x      = msg.linear.x
    self.ft_right_y      = msg.linear.y
    if( msg.linear.z > 0.1 ):
      self.ft_right_lift   = True
    else:
      self.ft_right_lift   = False

    if( msg.angular.x > 0.1 ):
      self.ft_right_lift_x   = True
    else:
      self.ft_right_lift_x   = False

    if( msg.angular.y > 0.1 ):
      self.ft_right_lift_y   = True
    else:
      self.ft_right_lift_y   = False

    return

  def cb_state_time(self, msg):
    self.time = msg.data
    return

  def cb_state_mot_pos(self, msg):
    for i in range(self.da.DARWIN_MOTOR_NUM + 1):
      self.motor_position[i] = msg.data[i]
    return

  def cb_state_mot_vel(self, msg):
    for i in range(self.da.DARWIN_MOTOR_NUM + 1):
      self.motor_velocity[i] = msg.data[i]
    return

  def cb_state_mot_vol(self, msg):
    for i in range(self.da.DARWIN_MOTOR_NUM + 1):
      self.motor_voltage[i] = msg.data[i]
    return

  def cb_state_mot_tor(self, msg):
    for i in range(self.da.DARWIN_MOTOR_NUM + 1):
      self.motor_torque[i] = msg.data[i]
    return

  def cb_state_mot_tmp(self, msg):
    for i in range(self.da.DARWIN_MOTOR_NUM + 1):
      self.motor_temperature[i] = msg.data[i]
    return


  def close(self):
    self.node.destroy_node()
    self.rclpy.shutdown()
    return


  def setMotDeg(self, mot=None, pos=None, vel=None, tor=None):
    if (pos != None):
      if (len(pos) > 0):
        posl = list(pos)
        for i in range(len(pos)):
          posl[i] = posl[i] / 180.0 * self.PI
        pos = tuple(posl)
    if (vel != None):
      if (len(vel) > 0):
        vell = list(vel)
        for i in range(len(vel)):
          vell[i] = vell[i] / 180.0 * self.PI
        vel = tuple(vell)

    return self.setMot(mot, pos, vel, tor)

  def setMot(self, mot=None, pos=None, vel=None, tor=None):
    if (mot == None):
      return 1
    if ( (pos == None) & (vel == None) & (tor == None) ):
      return 1

    ret = 1

    msg_cmd      = self.String()
    msg_cmd.data = 'post'
    
    msg      = self.String()
    # r is for rad
    msg.data = 'r '
    if (pos != None):
     if (len(mot) == len(pos)):
       for i in range (len(mot)):
          msg.data += str(int(mot[i])) + ' ' + str(float(pos[i])) + ' '
       self.pub_ref.publish(msg)
       self.pub_cmd.publish(msg_cmd)
       ret = 0

    msg = self.String()
    # r is for rad/sec
    msg.data = 'r '
    if (vel != None):
     if (len(mot) == len(vel)):
       for i in range (len(mot)):
          msg.data += str(int(mot[i])) + ' ' + str(float(vel[i])) + ' '
       self.pub_vel.publish(msg)
       self.pub_cmd.publish(msg_cmd)
       ret = 0

  
    msg = self.String()
    # p is for percent from 0.0 to 1.0
    msg.data = 'p '
    if (tor != None):
     if (len(mot) == len(tor)):
       for i in range (len(mot)):
          msg.data += str(int(mot[i])) + ' ' + str(float(tor[i])) + ' '
       self.pub_tor.publish(msg)
       self.pub_cmd.publish(msg_cmd)
       ret = 0

    return ret


  

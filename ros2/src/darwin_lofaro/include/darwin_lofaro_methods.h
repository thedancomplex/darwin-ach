using namespace std::chrono_literals;

/*
#define ENUM_FT_LEFT                       0
#define ENUM_FT_RIGHT                      1

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

#include "lofaro_utils_ros2.h"
#include "darwin_lofaro_methods_cmd.h"
#include "darwin_lofaro_methods_ref_pos.h"
#include "darwin_lofaro_methods_ref_vel.h"
#include "darwin_lofaro_methods_ref_tor.h"
#include "darwin_lofaro_methods_rates.h"

#define DARWIN_MOT_MIN 1
#define DARWIN_MOT_MAX 20

#define DARWIN_REF_POS_0 0.0
#define DARWIN_REF_VEL_0 0.75
#define DARWIN_REF_TOR_0 0.5
*/


DarwinLofaroLegacyRos2::DarwinLofaroLegacyRos2() : Node("darwin_lofaro_legacy_daemon")
{

//  this->dl = (DarwinLofaro*)malloc(sizeof(DarwinLofaro));
//  this->dl = new DarwinLofaro();

  for( int i = DARWIN_MOT_MIN; i <= DARWIN_MOT_MAX; i++)
  {
    this->dl->setMotPos(i,    DARWIN_REF_POS_0);
    this->dl->setMotSpeed(i,  DARWIN_REF_VEL_0);
    this->dl->setMotTorque(i, DARWIN_REF_TOR_0);
  }

  subscription_ref_pos_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS,         10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_pos, this, _1));

  subscription_ref_vel_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL,         10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_vel, this, _1));

  subscription_ref_tor_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR,         10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_tor, this, _1));

  subscription_cmd_         = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_CMD,             10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_cmd, this, _1));

  subscription_clock_       = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_CLOCK,           10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_clock, this, _1));

  publisher_state_imu_      = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_IMU,      1);

  publisher_state_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT,  1);

  publisher_state_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT, 1);

  publisher_state_motor_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_POS, 1);
  publisher_state_motor_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VEL, 1);
  publisher_state_motor_tor_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TOR, 1);
  publisher_state_motor_vol_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VOL, 1);
  publisher_state_motor_tmp_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TMP, 1);

//  timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
//  timer_ = this->create_wall_timer(8ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
  //timer_ = this->create_wall_timer(20ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
}


void DarwinLofaroLegacyRos2::timer_callback_main_loop()
{
 if(this->started)
 {
  auto buff_imu       = geometry_msgs::msg::Twist();
  auto buff_ft_left   = geometry_msgs::msg::Twist();
  auto buff_ft_right  = geometry_msgs::msg::Twist();
  auto buff_motor_pos = std_msgs::msg::Float64MultiArray();
  auto buff_motor_vel = std_msgs::msg::Float64MultiArray();
  auto buff_motor_tor = std_msgs::msg::Float64MultiArray();
  auto buff_motor_vol = std_msgs::msg::Float64MultiArray();
  auto buff_motor_tmp = std_msgs::msg::Float64MultiArray();

/*
  buff_motor_pos.data = (float*)malloc(sizeof(float) * (DARWIN_MOTOR_MAX + 1));
  buff_motor_vel.data = (float*)malloc(sizeof(float) * (DARWIN_MOTOR_MAX + 1));
  buff_motor_tor.data = (float*)malloc(sizeof(float) * (DARWIN_MOTOR_MAX + 1));
  buff_motor_vol.data = (float*)malloc(sizeof(float) * (DARWIN_MOTOR_MAX + 1));
  buff_motor_tmp.data = (float*)malloc(sizeof(float) * (DARWIN_MOTOR_MAX + 1));
  free(buff_motor_pos.data);
  free(buff_motor_vel.data);
  free(buff_motor_tor.data);
  free(buff_motor_vol.data);
  free(buff_motor_tmp.data);

  float buff_motor_pos_[DARWIN_MOTOR_MAX +1];
  float buff_motor_vel_[DARWIN_MOTOR_MAX +1];
  float buff_motor_tor_[DARWIN_MOTOR_MAX +1];
  float buff_motor_vol_[DARWIN_MOTOR_MAX +1];
  float buff_motor_tmp_[DARWIN_MOTOR_MAX +1];

*/

/*
  int ret = 0;

  // Set Ref 
  ret += this->dl->stageMotor();
  ret += this->dl->putMotor();

  // Get State 
  ret += this->dl->getImu();
  ret += this->dl->getFt();

  ret += this->dl->getMotorSlow(1);

*/
    int ret = 0;
    if     ( mode == RATE_50HZ )              ret += this->update_50hz();
    else if( mode == RATE_100HZ )             ret += this->update_100hz(HZ_MODE_MOTORS);
    else if( mode == RATE_100HZ_MOTOR_STATE ) ret += this->update_100hz(HZ_MODE_MOTORS_AND_STATE);
    else if( mode == RATE_125HZ )             ret += this->update_125hz();
    else                                      ret += this->update_100hz(HZ_MODE_MOTORS);


/*
    buff_motor_pos.layout.dim = DARWIN_MOTOR_MAX + 1;
    buff_motor_vel.layout.dim = DARWIN_MOTOR_MAX + 1;
    buff_motor_tor.layout.dim = DARWIN_MOTOR_MAX + 1;
    buff_motor_vol.layout.dim = DARWIN_MOTOR_MAX + 1;
    buff_motor_tmp.layout.dim = DARWIN_MOTOR_MAX + 1;
*/

//  for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
  {
    buff_motor_pos.data.push_back(this->dl->darwin_data.motor_state[i].pos);
    buff_motor_vel.data.push_back(this->dl->darwin_data.motor_state[i].speed);
    buff_motor_tor.data.push_back(this->dl->darwin_data.motor_state[i].load);
    buff_motor_vol.data.push_back(this->dl->darwin_data.motor_state[i].voltage);
    buff_motor_tmp.data.push_back(this->dl->darwin_data.motor_state[i].temp);
/*
    buff_motor_pos.data[i] = this->dl->darwin_data.motor_state[i].pos;
    buff_motor_vel.data[i] = this->dl->darwin_data.motor_state[i].speed;
    buff_motor_tor.data[i] = this->dl->darwin_data.motor_state[i].load;
    buff_motor_vol.data[i] = this->dl->darwin_data.motor_state[i].voltage;
    buff_motor_tmp.data[i] = this->dl->darwin_data.motor_state[i].temp;
*/
  }

/*
  buff_motor_pos.data = buff_motor_pos_;
  buff_motor_vel.data = buff_motor_vel_;
  buff_motor_tor.data = buff_motor_tor_;
  buff_motor_vol.data = buff_motor_vol_;
  buff_motor_tmp.data = buff_motor_tmp_;
*/

  buff_imu.linear.x  =         this->dl->darwin_data.imu.acc_x;
  buff_imu.linear.y  =         this->dl->darwin_data.imu.acc_y;
  buff_imu.linear.z  =         this->dl->darwin_data.imu.acc_z;
  buff_imu.angular.x =         this->dl->darwin_data.imu.gyro_x;
  buff_imu.angular.y =         this->dl->darwin_data.imu.gyro_y;
  buff_imu.angular.z =         this->dl->darwin_data.imu.gyro_z;

  int id = ENUM_FT_LEFT;
  buff_ft_left.linear.x  =     this->dl->darwin_data.ft[id].x;
  buff_ft_left.linear.y  =     this->dl->darwin_data.ft[id].y;
  buff_ft_left.linear.z  = (  (this->dl->darwin_data.ft[id].raised_x) | (this->dl->darwin_data.ft[id].raised_y)  );
  buff_ft_left.angular.x =     this->dl->darwin_data.ft[id].raised_x ;
  buff_ft_left.angular.y =     this->dl->darwin_data.ft[id].raised_y;


  id = ENUM_FT_RIGHT;
  buff_ft_right.linear.x  =    this->dl->darwin_data.ft[id].x;
  buff_ft_right.linear.y  =    this->dl->darwin_data.ft[id].y;
  buff_ft_right.linear.z  = ( (this->dl->darwin_data.ft[id].raised_x) | (this->dl->darwin_data.ft[id].raised_y) );
  buff_ft_right.angular.x =    this->dl->darwin_data.ft[id].raised_x ;
  buff_ft_right.angular.y =    this->dl->darwin_data.ft[id].raised_y;

  publisher_state_imu_->publish(buff_imu);
  publisher_state_ft_left_->publish(buff_ft_left);
  publisher_state_ft_right_->publish(buff_ft_right);
  publisher_state_motor_pos_->publish(buff_motor_pos);
  publisher_state_motor_vel_->publish(buff_motor_vel);
  publisher_state_motor_tor_->publish(buff_motor_tor);
  publisher_state_motor_vol_->publish(buff_motor_vol);
  publisher_state_motor_tmp_->publish(buff_motor_tmp);

 }
 return;
}


void DarwinLofaroLegacyRos2::topic_callback_clock(const std_msgs::msg::String & msg)
{
  timer_callback_main_loop();
 // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

/*
void DarwinLofaroLegacyRos2::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void DarwinLofaroLegacyRos2::topic_callback_ref_vel(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void DarwinLofaroLegacyRos2::topic_callback_ref_tor(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void DarwinLofaroLegacyRos2::topic_callback_cmd(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

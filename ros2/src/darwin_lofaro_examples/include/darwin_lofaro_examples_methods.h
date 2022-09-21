using namespace std::chrono_literals;

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

DarwinLofaroLegacyRos2Example::DarwinLofaroLegacyRos2Example() : Node("darwin_lofaro_interface")
{ 
  /* Zero out the data */
  memset(&this->darwin_data, 0, sizeof(this->darwin_data));

  subscription_state_imu_       = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_IMU,     1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_imu_,      this, _1));
  subscription_state_ft_left_   = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT, 1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_ft_left_,  this, _1));
  subscription_state_ft_right_  = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_ft_right_, this, _1));
  subscription_state_motor_pos_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_POS,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_motor_pos_, this, _1));
  subscription_state_motor_vel_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VEL,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_motor_vel_, this, _1));
  subscription_state_motor_tor_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TOR,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_motor_tor_, this, _1));
  subscription_state_motor_vol_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VOL,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_motor_vol_, this, _1));
  subscription_state_motor_tmp_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TMP,1, std::bind(&DarwinLofaroLegacyRos2Example::topic_callback_state_motor_tmp_, this, _1));

  publisher_ref_pos_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS, 1);
  publisher_ref_vel_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL, 1);
  publisher_ref_tor_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR, 1);
  publisher_cmd_     = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_CMD,     1);
  publisher_clock_   = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_CLOCK,   1);

}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_imu_(const geometry_msgs::msg:Twist & msg)
{
  this->darwin_data.imu.acc_x = msg.linear.x;
  this->darwin_data.imu.acc_y = msg.linear.y;
  this->darwin_data.imu.acc_z = msg.linear.z;
  this->darwin_data.imu.gyro_x = msg.angular.x;
  this->darwin_data.imu.gyro_y = msg.angular.y;
  this->darwin_data.imu.gyro_z = msg.angular.z;
  return;
}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_left_(const geometry_msgs::msg:Twist & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_right_(const geometry_msgs::msg:Twist & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_pos_(const std_msgs::msg::Float64MultiArray & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vel_(const std_msgs::msg::Float64MultiArray & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tor_(const std_msgs::msg::Float64MultiArray & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vol_(const std_msgs::msg::Float64MultiArray & msg){return;}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tmp_(const std_msgs::msg::Float64MultiArray & msg){return;}

void DarwinLofaroLegacyRos2Examples::set_motor_pos(int mot, double val)
{
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_pos_->publish(buff);
  return;
}

void DarwinLofaroLegacyRos2Examples::set_motor_vel(int mot, double val)
{
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_vel_->publish(buff);
  return;
}

void DarwinLofaroLegacyRos2Examples::set_motor_tor(int mot, double val)
{
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_tor_->publish(buff);
  return;
}


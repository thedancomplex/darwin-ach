using namespace std::chrono_literals;

#include "lofaro_utils_ros2.h"
#include "ros2_ach_bridge_methods_ref_pos.h"
#include "ros2_ach_bridge_methods_ref_vel.h"
#include "ros2_ach_bridge_methods_ref_tor.h"
#include "ros2_ach_bridge_methods_cmd.h"


Ros2AchBridge::Ros2AchBridge() : Node("darwin_lofaro_legacy_daemon")
{

//  this->dl = (DarwinLofaro*)malloc(sizeof(DarwinLofaro));
//  this->dl = new DarwinLofaro();

  for( int i = DARWIN_MOT_MIN; i <= DARWIN_MOT_MAX; i++)
  {
    this->dac.stageRefPos(   i, DARWIN_REF_POS_0);
    this->dac.stageRefVel(   i, DARWIN_REF_VEL_0);
    this->dac.stageRefTorque(i, DARWIN_REF_TOR_0);
  }
  this->dac.postRef();

  subscription_ref_pos_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS,         10, std::bind(&Ros2AchBridge::topic_callback_ref_pos, this, _1));

  subscription_ref_vel_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL,         10, std::bind(&Ros2AchBridge::topic_callback_ref_vel, this, _1));

  subscription_ref_tor_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR,         10, std::bind(&Ros2AchBridge::topic_callback_ref_tor, this, _1));

  subscription_cmd_         = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_CMD,             10, std::bind(&Ros2AchBridge::topic_callback_cmd, this, _1));

  publisher_state_imu_      = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_IMU,      1);

  publisher_state_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT,  1);

  publisher_state_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT, 1);

  publisher_state_motor_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_POS, 1);
  publisher_state_motor_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VEL, 1);
  publisher_state_motor_tor_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TOR, 1);
  publisher_state_motor_vol_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VOL, 1);
  publisher_state_motor_tmp_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TMP, 1);

}

/*
void Ros2AchBridge::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_ref_vel(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_ref_tor(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_cmd(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

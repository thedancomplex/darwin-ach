#include "lofaro_darwin.h"

using namespace std::chrono_literals;


#define DARWIN_TOPIC_REF_POS         "/darwin/ref/position"
#define DARWIN_TOPIC_REF_VEL         "/darwin/ref/speed"
#define DARWIN_TOPIC_REF_TOR         "/darwin/ref/torque"
#define DARWIN_TOPIC_STATE_IMU       "/darwin/state/imu"
#define DARWIN_TOPIC_STATE_FT_LEFT   "/darwin/state/ft/left"
#define DARWIN_TOPIC_STATE_FT_RIGHT  "/darwin/state/ft/right"

DarwinLofaroLegacyRos2::DarwinLofaroLegacyRos2() : Node("darwin_lofaro_legacy_daemon")
{
  subscription_ref_pos_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_pos, this, _1));

  subscription_ref_vel_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_vel, this, _1));

  subscription_ref_tor_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_tor, this, _1));

publisher_state_imu_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_STATE_IMU, 1);

publisher_state_ft_left_  = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_STATE_FT_LEFT, 1);

publisher_state_ft_right_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_STATE_FT_RIGHT, 1);

timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));


}


void DarwinLofaroLegacyRos2::timer_callback_main_loop()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_state_imu_->publish(message);
  publisher_state_ft_left_->publish(message);
  publisher_state_ft_right_->publish(message);
}




const void DarwinLofaroLegacyRos2::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

const void DarwinLofaroLegacyRos2::topic_callback_ref_vel(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

const void DarwinLofaroLegacyRos2::topic_callback_ref_tor(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

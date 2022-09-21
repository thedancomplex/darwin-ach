#define DARWIN_LOFARO_DYN 1
#include <lofaro_darwin.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class DarwinLofaroLegacyRos2Example : public rclcpp::Node
{
  public:
    DarwinLofaroLegacyRos2Example();
    DarwinLofaro::darwin_data_def_t darwin_data;
    void set_motor_pos(int mot, double val);
    void set_motor_vel(int mot, double val);
    void set_motor_tor(int mot, double val);

  private:
    void topic_callback_state_imu_(const geometry_msgs::msg::Twist & msg);
    void topic_callback_state_ft_left_(const geometry_msgs::msg::Twist & msg);
    void topic_callback_state_ft_right_(const geometry_msgs::msg::Twist & msg);
    void topic_callback_state_motor_pos_(const std_msgs::msg::Float64MultiArray & msg);
    void topic_callback_state_motor_vel_(const std_msgs::msg::Float64MultiArray & msg);
    void topic_callback_state_motor_tor_(const std_msgs::msg::Float64MultiArray & msg);
    void topic_callback_state_motor_vol_(const std_msgs::msg::Float64MultiArray & msg);
    void topic_callback_state_motor_tmp_(const std_msgs::msg::Float64MultiArray & msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_state_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_state_ft_left_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_state_ft_right_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_state_motor_pos_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_state_motor_vel_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_state_motor_tor_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_state_motor_vol_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_state_motor_tmp_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ref_pos_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ref_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ref_tor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_clock_;
//    DarwinLofaro* dl = new DarwinLofaro();
    /* State and Reference Data */
};


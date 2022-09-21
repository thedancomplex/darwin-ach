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

#define RATE_100HZ             0
#define RATE_125HZ             1
#define RATE_50HZ              2
#define RATE_100HZ_MOTOR_STATE 3

using std::placeholders::_1;

class DarwinLofaroLegacyRos2 : public rclcpp::Node
{
  public:
    DarwinLofaroLegacyRos2();

  private:
    void topic_callback_ref_pos(const std_msgs::msg::String & msg);
    void topic_callback_ref_vel(const std_msgs::msg::String & msg);
    void topic_callback_ref_tor(const std_msgs::msg::String & msg);
    void topic_callback_cmd(const std_msgs::msg::String & msg);
    void topic_callback_clock(const std_msgs::msg::String & msg);
    void timer_callback_main_loop();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_pos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_tor_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_ft_right_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_pos_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_tor_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_vol_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_tmp_;

    int update_50hz();
    int update_100hz();
    int update_100hz(int mode);
    int update_125hz();
    int mode = RATE_100HZ;

    size_t count_;
    bool started = false;
    DarwinLofaro* dl = new DarwinLofaro();
};


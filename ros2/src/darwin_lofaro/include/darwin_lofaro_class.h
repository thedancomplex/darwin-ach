#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;




class DarwinLofaroLegacyRos2 : public rclcpp::Node
{
  public:
    DarwinLofaroLegacyRos2();

  private:
    const void topic_callback_ref_pos(const std_msgs::msg::String & msg);
    const void topic_callback_ref_vel(const std_msgs::msg::String & msg);
    const void topic_callback_ref_tor(const std_msgs::msg::String & msg);
    void timer_callback_main_loop();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_pos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_tor_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_state_imu_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_state_ft_left_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_state_ft_right_;
    size_t count_;
};

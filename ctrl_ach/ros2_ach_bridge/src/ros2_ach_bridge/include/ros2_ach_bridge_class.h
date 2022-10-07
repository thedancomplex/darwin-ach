#include <lofaro_darwin_ach_client.h>
#include <lofaro_defines_ros2.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

#define MODE_BRIDGE_REF    1
#define MODE_BRIDGE_STATE  2
#define MODE_BRIDGE_COUNT  3

class Ros2AchBridge : public rclcpp::Node
{
  public:
    Ros2AchBridge(int mode);
    int state_loop();

  private:
    void topic_callback_ref_pos(const std_msgs::msg::String & msg);
    void topic_callback_ref_vel(const std_msgs::msg::String & msg);
    void topic_callback_ref_tor(const std_msgs::msg::String & msg);
    void topic_callback_cmd(const std_msgs::msg::String & msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_pos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_tor_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_cmd_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_state_ft_right_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_pos_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_tor_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_vol_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_motor_tmp_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_time_;
    bool do_debug = false;
    bool started = false;
    DarwinAchClient dac = DarwinAchClient(true);

    
};


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

/* Darwin-Legacy */
#include <lofaro_darwin.h>
#include <lofaro_utils_ros2.h>
using std::placeholders::_1;


class DarwinLofaro : public rclcpp::Node
{
  public:
    #define ENUM_FT_LEFT 0
    #define ENUM_FT_RIGHT 1
    #define M_PI 3.14159265358979323846

    int setupSubscription();

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ref_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_right_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_com_;

    int getRef(const std_msgs::msg::String & msg);
    void theLoop();
    int getCmd(const std_msgs::msg::String & msg);
};


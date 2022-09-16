#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "darwin_lofaro_msgs/msg/motor_ref_stamped.hpp"
//#include "darwin_lofaro_msgs/msg/string.hpp"
//#include "darwin_lofaro_msgs/msg/motor_ref.hpp"
//#include "darwin_lofaro_msgs/msg/MotorRefStamped.hpp"

using namespace std::chrono_literals;

/* Darwin-Legacy */
#include "lofaro_darwin.h"
#include <lofaro_utils_ros2.h>

using std::placeholders::_1;

class DarwinLofaroRef : public rclcpp::Node
{

  #define M_PI 3.14159265358979323846

  public:
    DarwinLofaroRef(std::shared_ptr<DarwinLofaro> d);

  private:
    std::shared_ptr<DarwinLofaro> dl;
    const void topic_callback(const std_msgs::msg::String & msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class DarwinLofaroState : public rclcpp::Node
{
  public:
    DarwinLofaroState(std::shared_ptr<DarwinLofaro> d);

  private:
    #define ENUM_FT_LEFT 0
    #define ENUM_FT_RIGHT 1
    std::shared_ptr<DarwinLofaro> dl;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_right_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_com_;
    size_t count_;
};


class DarwinLofaroCmd : public rclcpp::Node
{
  public:
    DarwinLofaroCmd(std::shared_ptr<DarwinLofaro> d);

  private:
    std::shared_ptr<DarwinLofaro> dl;
    const void topic_callback(const std_msgs::msg::String & msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class DarwinLofaroLoop : public rclcpp::Node
{

  public:
    DarwinLofaroLoop(std::shared_ptr<DarwinLofaro> d, rclcpp::executors::MultiThreadedExecutor *exec);

  private:
    std::shared_ptr<DarwinLofaro> dl;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_right_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_com_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    const void topic_callback(const std_msgs::msg::String & msg);
    void theLoop();
    void timerLoop();
    void loop(rclcpp::executors::MultiThreadedExecutor *exec);
    void rtLoop();
    void rtLoopSetup();

};


class DarwinData
{
  public:
    darwin::darwin_data_def_t darwin_data;
    darwin::imu_state_def_t   imu_state;
    darwin::ft_state_def_t    ft_state;
    darwin::motor_state_def_t motor_state;
    darwin::motor_ref_def_t   motor_ref;
    DarwinData()
    {
      memset(&darwin_data,  0, sizeof(darwin_data));
      memset(&motor_ref,    0, sizeof(motor_ref));
      memset(&motor_state,  0, sizeof(motor_state));
      memset(&ft_state,     0, sizeof(ft_state));
      memset(&imu_state,    0, sizeof(imu_state));
    }
};

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
#include <lofaro_darwin.h>

using std::placeholders::_1;

class DarwinLofaroRef : public rclcpp::Node
{
  public:
    DarwinLofaroRef() : Node("darwin_lofaro_ref_subscriber")
    {
      /*
        const char* head = "/darwin/ref";
        std::string s = std::to_string(i);
        const char* id = s.c_str();
        char* top = new char[strlen(head) + strlen(id) + 1];
        strcpy(top, head);
        strcat(top, id);
      */
        const char* top = "/darwin/ref";
        subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class DarwinLofaroState : public rclcpp::Node
{
  public:
    DarwinLofaroState()
    : Node("darwin_lofaro_state_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/imu", 10);
      //publisher_ = this->create_publisher<std_msgs::msg::String>("/darwin/ref", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&DarwinLofaroState::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.0;
      message.linear.y = 2.0;
      message.linear.z = 3.0;
      message.angular.x = 4.0;
      message.angular.y = 5.0;
      message.angular.z = 6.0;
//      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};


class DarwinLofaroCmd : public rclcpp::Node
{
  public:
    DarwinLofaroCmd() : Node("darwin_lofaro_cmd_subscriber")
    {
        const char* top = "/darwin/cmd";
        subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroCmd::topic_callback, this, _1));
    }

  private:

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      std::string str_msg = msg.data;
      std::string str_on  ("on all");
      std::string str_off ("off all");

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro::do_split(str_msg, delim, dout);

      int i = 0;
      for(auto &str_msg: dout)
      {
         RCLCPP_INFO(this->get_logger(), "Message[%d]: %s", i, dout[i].c_str());
         i++;
      }
      RCLCPP_INFO(this->get_logger(), "Message Length: '%d'", i);


      if     ( str_msg.compare(str_on) == 0 )
      {
        RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy");
        darwin::setup("/dev/ttyUSB0");
        darwin::on();
      }
      else if( str_msg.compare(str_off) == 0 )
      {
        RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy");
        darwin::off();
      }
      RCLCPP_INFO(this->get_logger(), "Message: '%s'", str_msg.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node_darwin_ref   = std::make_shared<DarwinLofaroRef>();
  auto node_darwin_state = std::make_shared<DarwinLofaroState>();
  auto node_darwin_cmd   = std::make_shared<DarwinLofaroCmd>();
  //std::shared_ptr<DarwinLofaroRef> node_darwin = std::make_shared<DarwinLofaroRef>(i);
  exec.add_node(node_darwin_ref);
  exec.add_node(node_darwin_state);
  exec.add_node(node_darwin_cmd);


  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

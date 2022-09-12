#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class DarwinLofaroClock : public rclcpp::Node
{
  public:
    DarwinLofaroClock() : Node("darwin_lofaro_clock_publisher"), count_(0)
    {
      publisher_clock_ = this->create_publisher<std_msgs::msg::String>("/darwin/clock", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroClock::theLoop, this));
    }


  private:
    void theLoop()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_clock_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_clock_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

//  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node_darwin_clock   = std::make_shared<DarwinLofaroClock>();
  exec.add_node(node_darwin_clock);

  printf("ROS about to spin\n");

  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<DarwinLofaroClock>());
  rclcpp::shutdown();
  return 0;
}

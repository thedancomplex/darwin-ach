#include <ros2_ach_bridge.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2AchBridge>());
  rclcpp::shutdown();
  return 0;
}


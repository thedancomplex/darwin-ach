#include <ros2_ach_bridge.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2AchBridge>(MODE_BRIDGE_REF));
  rclcpp::shutdown();
  return 0;
}


#include <ros2_ach_bridge.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  Ros2AchBridge rab = Ros2AchBridge(MODE_BRIDGE_STATE);
  while(true)
  {
    rab.state_loop();
  }
  rclcpp::shutdown();
  return 0;
}


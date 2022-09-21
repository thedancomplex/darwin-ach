#include <darwin_lofaro.h>
//#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "/usr/local/include/dynamixel_sdk/dynamixel_sdk.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DarwinLofaroLegacyRos2>());
  rclcpp::shutdown();
  return 0;
}


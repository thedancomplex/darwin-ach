#include "darwin_lofaro.h"

int main(int argc, char * argv[])
{
/*
  printf("Setting up Darwin-Lofaro Legacy\n");
  darwin::setup("/dev/ttyUSB0");
  darwin::sleep(2.0);
  printf("Turning on actuators\n");
  darwin::on();
  darwin::sleep(2.0);
  printf("Setting up ROS\n");
*/
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  //rclcpp::executors::SingleThreadedExecutor exec;

  auto node_darwin_ref   = std::make_shared<DarwinLofaroRef>();
  auto node_darwin_state = std::make_shared<DarwinLofaroState>();
  auto node_darwin_cmd   = std::make_shared<DarwinLofaroCmd>();
  //std::shared_ptr<DarwinLofaroRef> node_darwin = std::make_shared<DarwinLofaroRef>(i);
  exec.add_node(node_darwin_ref);
  exec.add_node(node_darwin_state);
  exec.add_node(node_darwin_cmd);

  printf("ROS about to spin\n");

  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

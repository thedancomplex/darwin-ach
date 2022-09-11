#include "darwin_lofaro.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

//  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::executors::SingleThreadedExecutor exec;

//  auto node_darwin_ref   = std::make_shared<DarwinLofaroRef>();
//  auto node_darwin_state = std::make_shared<DarwinLofaroState>();
//  auto node_darwin_cmd   = std::make_shared<DarwinLofaroCmd>();

  auto node_darwin_daemon   = std::make_shared<DarwinLofaro>();

//  exec.add_node(node_darwin_ref);
//  exec.add_node(node_darwin_state);
//  exec.add_node(node_darwin_cmd);

  exec.add_node(node_darwin_daemon);

  printf("ROS about to spin\n");

  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

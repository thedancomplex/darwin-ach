#include "darwin_lofaro.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

//  DarwinData::DarwinData darwin_data = new DarwinData::DarwinData();


  /* Main Darwin Lofaro Legacy Object */
//  DarwinLofaro dl = DarwinLofaro();
//  std::shared_ptr<DarwinLofaro>
  std::shared_ptr<DarwinLofaro> dl = std::make_shared<DarwinLofaro>();

  rclcpp::executors::MultiThreadedExecutor exec;
  //rclcpp::executors::SingleThreadedExecutor exec;

  auto node_darwin_ref   = std::make_shared<DarwinLofaroRef>(dl);
  auto node_darwin_state = std::make_shared<DarwinLofaroState>(dl);
  auto node_darwin_cmd   = std::make_shared<DarwinLofaroCmd>(dl);
  auto node_darwin_loop  = std::make_shared<DarwinLofaroLoop>(dl, &exec);
  //std::shared_ptr<DarwinLofaroRef> node_darwin = std::make_shared<DarwinLofaroRef>(i);
  exec.add_node(node_darwin_ref);
  exec.add_node(node_darwin_state);
  exec.add_node(node_darwin_cmd);
  exec.add_node(node_darwin_loop);

  printf("ROS about to spin\n");

/*
  while(true)
  {
    printf("----------------------------\n");
    printf("%f, %f, %f\n", darwin::imu_acc_x, darwin::imu_acc_y, darwin::imu_acc_z);
    darwin::sleep(0.01);
  }
*/
  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

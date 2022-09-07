#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* Darwin-Legacy */
#include <lofaro_darwin.h>

using std::placeholders::_1;

class DarwinLofaroRef : public rclcpp::Node
{
  public:
    DarwinLofaroRef(int i)
    : Node("darwin_lofaro_ref_subscriber")
    {
      const char* head = "/darwin/ref/joint";
      std::string s = std::to_string(i);
      const char* id = s.c_str();
      char* top = new char[strlen(head) + strlen(id) + 1];
      strcpy(top, head);
      strcat(top, id);
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  std::shared_ptr<DarwinLofaroRef> node_darwin[20];
  for( int i = 0; i < 20; i++ )
  {
    node_darwin[i] = std::make_shared<DarwinLofaroRef>(i);
    exec.add_node(node_darwin[i]);
  }
  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

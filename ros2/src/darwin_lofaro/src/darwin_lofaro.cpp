#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "darwin_lofaro_msgs/msg/motor_ref_stamped.hpp"
//#include "darwin_lofaro_msgs/msg/MotorRefStamped.hpp"


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
        subscription_ = this->create_subscription<darwin_lofaro_msgs::msg::MotorRefStamped>(top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
        //subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
    }

  private:
    void topic_callback(const darwin_lofaro_msgs::msg::MotorRefStamped & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.id);
    }
    rclcpp::Subscription<darwin_lofaro_msgs::msg::MotorRefStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node_darwin = std::make_shared<DarwinLofaroRef>();
  //std::shared_ptr<DarwinLofaroRef> node_darwin = std::make_shared<DarwinLofaroRef>(i);
  exec.add_node(node_darwin);


  //std::shared_ptr node1 = std::make_shared<DarwinLofaroRef>(1);
  //auto node1 = std::make_shared<DarwinLofaroRef>(1);
  exec.spin();
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

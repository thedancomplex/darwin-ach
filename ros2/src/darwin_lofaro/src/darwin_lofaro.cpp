#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "darwin_lofaro_msgs/msg/motor_ref_stamped.hpp"
//#include "darwin_lofaro_msgs/msg/string.hpp"
//#include "darwin_lofaro_msgs/msg/motor_ref.hpp"
//#include "darwin_lofaro_msgs/msg/MotorRefStamped.hpp"

using namespace std::chrono_literals;

/* Darwin-Legacy */
#include <lofaro_darwin.h>
#include <lofaro_utils_ros2.h>
using std::placeholders::_1;


class DarwinLofaroRef : public rclcpp::Node
{

  #define M_PI 3.14159265358979323846

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
        subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
  //    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data);
      std::string str_msg = msg.data;

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro_utils_ros2::do_split(str_msg, delim, dout);

      int i = 0;
      int length = dout.size();

      if( length     <  3 ) return;
      if( (length%2) != 1 ) return;
//      printf("length = %d\n", length);

      
      std::string s_head = dout[0];
      double scale = 1.0;
      if     ( s_head.compare("d") == 0 )
      { 
         scale = M_PI / 180.0;
      }
      else if( s_head.compare("r") == 0 )
      {
         scale = 1.0;
      }
      else
      {
         return;
      }

      for( int i = 1; i < length; i = i + 2 )
      { 
//        printf("i = %d\n",i);
        try
        {
         std::string s0 = dout[i];
         std::string s1 = dout[i+1];
         int    mot_num = std::stoi(s0);
         double mot_val = std::stod(s1) * scale;

         if(mot_val >  (M_PI / 2.0)) return;
         if(mot_val < -(M_PI / 2.0)) return;

         darwin::motor_ref[mot_num] = mot_val;
         throw 1;
        }
        catch(...){}
      }

      return;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class DarwinLofaroState : public rclcpp::Node
{
  public:
    DarwinLofaroState()
    : Node("darwin_lofaro_state_publisher"), count_(0)
    {
      publisher_imu_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/imu", 10);
      publisher_ft_left_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/left", 10);
      publisher_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/right", 10);
      publisher_ft_com_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/com", 10);
      //publisher_ = this->create_publisher<std_msgs::msg::String>("/darwin/ref", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&DarwinLofaroState::timer_callback, this));
    }

  private:
    #define ENUM_FT_LEFT 0
    #define ENUM_FT_RIGHT 1
    void timer_callback()
    {

      // read 1 byte from address 5
      //lofaro::do_read(200, 3);
      for (int i = 1; i <= 20; i++)
      {
//      darwin::set_motor_pos(i, 0.0);
        darwin::set_motor_pos_set(i, darwin::motor_ref[i]);
      }
      darwin::write_send();

      darwin::get_imu_state_auto();
      darwin::get_ft_state_auto();
      darwin::get_motor_state_auto(1);
      darwin::sleep(0.001);

      bool do_loop = true;
      double tick = darwin::time();
      double tock = darwin::time();
      while(do_loop)
      {
        tock = darwin::time();
        double dt = tock - tick;
        if (dt > 0.001) do_loop = false;
        darwin::read_buffer();
        darwin::sleep(0.0001);
      }

      auto message_imu          = geometry_msgs::msg::Twist();
      auto message_ft_left      = geometry_msgs::msg::Twist();
      auto message_ft_right     = geometry_msgs::msg::Twist();
      auto message_ft_com       = geometry_msgs::msg::Twist();

      message_imu.linear.x      = darwin::imu_acc_x;
      message_imu.linear.y      = darwin::imu_acc_y;
      message_imu.linear.z      = darwin::imu_acc_z;
      message_imu.angular.x     = darwin::imu_gyro_x;
      message_imu.angular.y     = darwin::imu_gyro_y;
      message_imu.angular.z     = darwin::imu_gyro_z;

      message_ft_left.linear.x  = darwin::ft_state[ENUM_FT_LEFT].x;
      message_ft_left.linear.y  = darwin::ft_state[ENUM_FT_LEFT].y;
      message_ft_left.linear.z  = darwin::ft_state[ENUM_FT_LEFT].raised_x | darwin::ft_state[ENUM_FT_LEFT].raised_y;

      message_ft_right.linear.x  = darwin::ft_state[ENUM_FT_RIGHT].x;
      message_ft_right.linear.y  = darwin::ft_state[ENUM_FT_RIGHT].y;
      message_ft_right.linear.z  = darwin::ft_state[ENUM_FT_RIGHT].raised_x | darwin::ft_state[ENUM_FT_RIGHT].raised_y;

//      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_imu_->publish(message_imu);
      publisher_ft_right_->publish(message_ft_right);
      publisher_ft_left_->publish(message_ft_left);
      publisher_ft_com_->publish(message_ft_com);

      darwin::flush_final();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_left_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_right_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ft_com_;
    size_t count_;
};


class DarwinLofaroCmd : public rclcpp::Node
{
  public:
    DarwinLofaroCmd() : Node("darwin_lofaro_cmd_subscriber")
    {
        const char* top = "/darwin/cmd";
        subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroCmd::topic_callback, this, _1));
    }

  private:

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      std::string str_msg = msg.data;

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro_utils_ros2::do_split(str_msg, delim, dout);

      int i = 0;
      int length = dout.size();
/*
      for(auto &str_msg: dout)
      { 
         RCLCPP_INFO(this->get_logger(), "Message[%d]: %s", i, dout[i].c_str());
         i++;
      }
*/
      if(length < 1) return;
      std::string s0 = dout[0];
      if( s0.compare("open") == 0 )
      {
         RCLCPP_INFO(this->get_logger(), "Darwin-Lofaro Legacy: Startup");
         darwin::setup("/dev/ttyUSB0", false);
         //darwin::setup("/dev/ttyUSB0", true);
         return;
      }
      else if( s0.compare("close") == 0 )
      {
         RCLCPP_INFO(this->get_logger(), "Darwin-Lofaro Legacy: Close");
         darwin::close();
         return;
      }
      else if( s0.compare("on") == 0 )
      {
           if(length < 2) return;
           std::string s1 = dout[1];
           if( s1.compare("all") == 0 )
           {
             /*
             RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy");
             darwin::setup("/dev/ttyUSB0");
             darwin::on();
             */
             printf("Setting up Darwin-Lofaro Legacy\n");
             darwin::setup("/dev/ttyUSB0", true);
             darwin::sleep(2.0);
             printf("Turning on actuators\n");
             darwin::on();
             darwin::sleep(2.0);
             printf("Actuators On\n");
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy Mot: %d", mot_num);
                darwin::on(mot_num);
                return;
                throw 1;
              }
              catch(...)
              {
                RCLCPP_INFO(this->get_logger(), "Bad CMD");
                return;
              }
           }
      }
      else if( s0.compare("off") == 0 )
      {
           if(length < 2) return;
           std::string s1 = dout[1];
           if( s1.compare("all") == 0 )
           {
             RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy");
             darwin::off();
             darwin::close();
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy Mot: %d", mot_num);
                darwin::off(mot_num);
                return;
                throw 1;
              }
              catch(...)
              {
                RCLCPP_INFO(this->get_logger(), "Bad CMD");
                return;
              }
           }
      }
  
  
      RCLCPP_INFO(this->get_logger(), "Message: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


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

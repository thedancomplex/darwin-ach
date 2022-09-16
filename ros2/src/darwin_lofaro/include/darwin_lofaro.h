#include "darwin_lofaro_head.h"


DarwinLofaroRef::DarwinLofaroRef(std::shared_ptr<DarwinLofaro> d) : Node("darwin_lofaro_ref_subscriber")
{
  dl = d;
  const char* top = "/darwin/ref";
  subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroRef::topic_callback, this, _1));
}

const void DarwinLofaroRef::topic_callback(const std_msgs::msg::String & msg)
{
//dan  try {
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

         dl.darwin_data.motor_ref[mot_num].pos = mot_val;
         throw 1;
        }
        catch(...){}
      }

      return;
//dan  } catch(...){}
}


DarwinLofaroState::DarwinLofaroState(std::shared_ptr<DarwinLofaro> d) : Node("darwin_lofaro_state_publisher"), count_(0)
{
  dl = d;
//dan  try {
      publisher_imu_      = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/imu", 10);
      publisher_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/left", 10);
      publisher_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/right", 10);
      publisher_ft_com_   = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/com", 10);
      //timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroState::theLoop, this));
      //publisher_ = this->create_publisher<std_msgs::msg::String>("/darwin/ref", 10);
//dan  } catch(...){}
}

DarwinLofaroLoop::DarwinLofaroLoop(std::shared_ptr<DarwinLofaro> d, rclcpp::executors::MultiThreadedExecutor *exec) : Node("darwin_lofaro_loop")
{
  dl = d;
  const char* top = "/darwin/clock";
  subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroLoop::topic_callback, this, _1));
  publisher_imu_      = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/imu", 10);
  publisher_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/left", 10);
  publisher_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/right", 10);
  publisher_ft_com_   = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/com", 10);
      //timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroState::theLoop, this));
//  loop(darwin_data, exec);
//  rtLoopSetup();
//  rtLoop();
//  timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLoop::timerLoop, this));
}

const void DarwinLofaroLoop::topic_callback(const std_msgs::msg::String & msg)
{
  this->theLoop();
}

void DarwinLofaroLoop::loop(rclcpp::executors::MultiThreadedExecutor *exec)
{
}

void DarwinLofaroLoop::timerLoop()
{
  this->theLoop();
}


#define DARWIN_MOTOR_MIN 1
#define DARWIN_MOTOR_MAX 20

void DarwinLofaroLoop::theLoop()
{
      dl.getImu();
      dl.getFt();

      for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
      {
        dl.setMotSpeed(i,  0.75);
        dl.setMotTorque(i, 0.5);
      }
      dl.stageMotor();
      dl.putMotor();

      auto message_imu          = geometry_msgs::msg::Twist();
      auto message_ft_left      = geometry_msgs::msg::Twist();
      auto message_ft_right     = geometry_msgs::msg::Twist();
      auto message_ft_com       = geometry_msgs::msg::Twist();

/*
      message_imu.linear.x      = dl.imu_acc_x;
      message_imu.linear.y      = dl.imu_acc_y;
      message_imu.linear.z      = dl.imu_acc_z;
      message_imu.angular.x     = dl.imu_gyro_x;
      message_imu.angular.y     = dl.imu_gyro_y;
      message_imu.angular.z     = dl.imu_gyro_z;
*/
      message_imu.linear.x      = dl.darwin_data.imu_state.acc_x;
      message_imu.linear.y      = dl.darwin_data.imu_state.acc_y;
      message_imu.linear.z      = dl.darwin_data.imu_state.acc_z;
      message_imu.angular.x     = dl.darwin_data.imu_state.gyro_x;
      message_imu.angular.y     = dl.darwin_data.imu_state.gyro_y;
      message_imu.angular.z     = dl.darwin_data.imu_state.gyro_z;

/*      message_ft_left.linear.x  = dl.ft_state[ENUM_FT_LEFT].x;
      message_ft_left.linear.y  = dl.ft_state[ENUM_FT_LEFT].y;
      message_ft_left.linear.z  = dl.ft_state[ENUM_FT_LEFT].raised_x | darwin::ft_state[ENUM_FT_LEFT].raised_y;

      message_ft_right.linear.x  = dl.ft_state[ENUM_FT_RIGHT].x;
      message_ft_right.linear.y  = dl.ft_state[ENUM_FT_RIGHT].y;
      message_ft_right.linear.z  = dl.ft_state[ENUM_FT_RIGHT].raised_x | darwin::ft_state[ENUM_FT_RIGHT].raised_y;
*/

      message_ft_left.linear.x  = dl.darwin_data.ft_state[ENUM_FT_LEFT].x;
      message_ft_left.linear.y  = dl.darwin_data.ft_state[ENUM_FT_LEFT].y;
      message_ft_left.linear.z  = dl.darwin_data.ft_state[ENUM_FT_LEFT].raised_x | dl.darwin_data.ft_state[ENUM_FT_LEFT].raised_y;

      message_ft_right.linear.x = dl.darwin_data.ft_state[ENUM_FT_RIGHT].x;
      message_ft_right.linear.y = dl.darwin_data.ft_state[ENUM_FT_RIGHT].y;
      message_ft_right.linear.z = dl.darwin_data.ft_state[ENUM_FT_RIGHT].raised_x | dl.darwin_data.ft_state[ENUM_FT_RIGHT].raised_y;

//      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_imu_->publish(message_imu);
      publisher_ft_right_->publish(message_ft_right);
      publisher_ft_left_->publish(message_ft_left);
      publisher_ft_com_->publish(message_ft_com);
}


DarwinLofaroCmd::DarwinLofaroCmd(std::shared_ptr<DarwinLofaro> d) : Node("darwin_lofaro_cmd_subscriber")
{
  dl = d;
  const char* top = "/darwin/cmd";
  subscription_ = this->create_subscription<std_msgs::msg::String>(top, 10, std::bind(&DarwinLofaroCmd::topic_callback, this, _1));
}

const void DarwinLofaroCmd::topic_callback(const std_msgs::msg::String & msg)
{
//dan  try {
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
         //dl.setup("/dev/ttyUSB0", false);
         dl.setup("/dev/ttyUSB0", true);
         return;
      }
      else if( s0.compare("close") == 0 )
      {
         RCLCPP_INFO(this->get_logger(), "Darwin-Lofaro Legacy: Close");
         dl.close();
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
             dl.setup("/dev/ttyUSB0");
             dl.on();
             */
             printf("Setting up Darwin-Lofaro Legacy\n");
             dl.setup("/dev/ttyUSB0", true);
             dl.sleep(2.0);
             printf("Turning on actuators\n");
             dl.on();
             dl.sleep(2.0);
             printf("Actuators On\n");
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy Mot: %d", mot_num);
                dl.on(mot_num);
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
             dl.off();
             dl.close();
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy Mot: %d", mot_num);
                dl.off(mot_num);
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
//  } catch(...){}

}






void DarwinLofaroLoop::rtLoopSetup()
{

}

void DarwinLofaroLoop::rtLoop()
{
}



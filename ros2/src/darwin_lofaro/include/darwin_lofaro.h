#include "darwin_lofaro_head.h"

int DarwinLofaro::setupSubscription()
    {
      try
      {
        rclcpp::WaitSet wait_set;
        const char* topRef = "/darwin/ref";
        subscription_ref_ = wait_set.add_subscription<std_msgs::msg::String>(topRef);
        const char* topCmd = "/darwin/cmd";
        subscription_cmd_ = wait_set.add_subscription<std_msgs::msg::String>(topCmd);
      }
      catch(...){}
      return 0;
    }
    int setupPublish()
    {
      try
        {
          publisher_imu_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/imu", 10);
          publisher_ft_left_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/left", 10);
          publisher_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/right", 10);
          publisher_ft_com_ = this->create_publisher<geometry_msgs::msg::Twist>("/darwin/ft/com", 10);
        }
      catch(...){}
      return 0;
    }

    int getRef(const std_msgs::msg::String & msg)
    {
      try
        {
        std::string str_msg = msg.data;

        const char delim = ' ';
        std::vector<std::string> dout;
        lofaro_utils_ros2::do_split(str_msg, delim, dout);

        int i = 0;
        int length = dout.size();

        if( length     <  3 ) return 1;
        if( (length%2) != 1 ) return 1;

      
        std::string s_head = dout[0];
        double scale = 1.0;
        if     ( s_head.compare("d") == 0 ){ scale = M_PI / 180.0; }
        else if( s_head.compare("r") == 0 ){ scale = 1.0; }
        else { return 1; }

        for( int i = 1; i < length; i = i + 2 )
        { 
          try
          {
           std::string s0 = dout[i];
           std::string s1 = dout[i+1];
           int    mot_num = std::stoi(s0);
           double mot_val = std::stod(s1) * scale;

           if(mot_val >  (M_PI / 2.0)) return 1;
           if(mot_val < -(M_PI / 2.0)) return 1;

           darwin::motor_ref[mot_num] = mot_val;
           throw 1;
          }
          catch(...){}
        }

        return 1;
      }
      catch(...){}
      return 0;
    }

void DarwinLofaro::theLoop()
  {
    try
      {

        for (int i = 1; i <= 20; i++)
        {
          darwin::set_motor_pos(i, 0.0);
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
    catch(...){}
    return 0;
  }


int DarwinLofaro::getCmd(const std_msgs::msg::String & msg) 
  {
    try
    {
      std::string str_msg = msg.data;

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro_utils_ros2::do_split(str_msg, delim, dout);

      int i = 0;
      int length = dout.size();

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
    catch(...){}
    return 0;
  }


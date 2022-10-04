void Ros2AchBridge::topic_callback_cmd(const std_msgs::msg::String & msg)
{
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());


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
         this->dac.cmd(DARWIN_CMD_OPEN, false);
         this->started = true;
         return;
      }
      else if( s0.compare("close") == 0 )
      {
         RCLCPP_INFO(this->get_logger(), "Darwin-Lofaro Legacy: Close");
         this->dac.cmd(DARWIN_CMD_CLOSE, false);
         this->started = false;
         return;
      }
      else if( s0.compare("on") == 0 )
      {
           if(length < 2) return;
           std::string s1 = dout[1];
           if( s1.compare("all") == 0 )
           {
             printf("Setting up Darwin-Ach using Darwin-Lofaro Legacy\n");
             this->dac.cmd(DARWIN_CMD_ON, false);
             this->dac.sleep(4.0);
             printf("Actuators On\n");
             this->started = true;
             return;
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy Mot: %d", mot_num);
                this->dac.cmd(DARWIN_CMD_ON_MOTOR, mot_num, false);
                this->dac.sleep(4.0);
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
             this->dac.cmd(DARWIN_CMD_OFF, false);
             this->dac.sleep(4.0);
             this->started = false;
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy Mot: %d", mot_num);
                this->dac.cmd(DARWIN_CMD_OFF_MOTOR, mot_num, false);
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





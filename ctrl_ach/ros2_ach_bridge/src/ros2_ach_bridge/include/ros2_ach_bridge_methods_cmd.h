void Ros2AchBridge::topic_callback_cmd(const std_msgs::msg::String & msg)
{
      if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"I heard: '%s'\n", msg.data.c_str()); }

      std::string str_msg = msg.data;

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro_utils_ros2::do_split(str_msg, delim, dout);

      int i = 0;
      int length = dout.size();
      if(length < 1) return;
      std::string s0 = dout[0];
      if( s0.compare("post") == 0 )
      {
         if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Darwin-Lofaro Legacy: Post Ref\n"); }
         this->dac.postRef();
         this->started = true;
         return;
      }
      else if( s0.compare("open") == 0 )
      {
         if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Darwin-Lofaro Legacy: Startup\n"); }
         this->dac.cmd(DARWIN_CMD_OPEN, false);
         this->started = true;
         return;
      }
      else if( s0.compare("close") == 0 )
      {
         if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Darwin-Lofaro Legacy: Close\n"); }
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
             if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Setting up Darwin-Ach using Darwin-Lofaro Legacy\n"); }
             this->dac.cmd(DARWIN_CMD_ON, false);
             this->dac.sleep(4.0);
             if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Actuators On\n"); }
             this->started = true;
             return;
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Turing on Darwin-Lofaro Legacy Mot: %d\n", mot_num); }
                this->dac.cmd(DARWIN_CMD_ON_MOTOR, mot_num, false);
                this->dac.sleep(4.0);
                return;
                throw 1;
              }
              catch(...)
              {
                if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Bad CMD\n"); }
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
             if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Turing off Darwin-Lofaro Legacy\n"); }
             this->dac.cmd(DARWIN_CMD_OFF, false);
             this->dac.sleep(4.0);
             this->started = false;
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Turing off Darwin-Lofaro Legacy Mot: %d\n", mot_num); }
                this->dac.cmd(DARWIN_CMD_OFF_MOTOR, mot_num, false);
                return;
                throw 1;
              }
              catch(...)
              {
                if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Bad CMD\n"); }
                return;
              }
           }
      }
      else if( s0.compare("debug") == 0 )
      {
           if(length < 2) return;
           std::string s1 = dout[1];
           if( s1.compare("true") == 0  ){ this->do_debug = true; }
           if( s1.compare("false") == 0 ){ this->do_debug = false; }
      }
      else if( s0.compare("loop") == 0 )
      {
           if(length < 3) return;
           std::string s1 = dout[1];
           std::string s2 = dout[2];
           if( s1.compare("state") == 0 )
           {
             if     ( s2.compare("50hz_imu_motor_ft") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_50_IMU_MOTOR_FT, false);
             }
             else if( s2.compare("50hz_imu") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_50_IMU, false);
             }
             else if( s2.compare("125hz_imu") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_125_IMU, false);
             }
             else if( s2.compare("100hz_imu_ft_slow") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_100_IMU_FT_SLOW, false);
             }
             else if( s2.compare("100hz_imu_motors_slow") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_100_IMU_MOTORS_SLOW, false);
             }
             else if( s2.compare("default") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_STATE_DEFAULT, false);
             }
           }
           else if ( s1.compare("ref") == 0 )
           {
             if     ( s2.compare("slow_top") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_REF_SLOW_TOP, false);
             }
             else if( s2.compare("normal") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_REF_NORMAL, false);
             }
             else if( s2.compare("default") == 0 )
             {
               this->dac.cmd(DARWIN_CMD_LOOP_MODE, HZ_REF_DEFAULT, false);
             }
           }
      }
  
      if(this->do_debug){ RCLCPP_INFO(this->get_logger(),"Message: '%s'", msg.data.c_str()); }
      return;
}





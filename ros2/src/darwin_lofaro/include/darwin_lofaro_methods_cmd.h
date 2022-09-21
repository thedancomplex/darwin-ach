void DarwinLofaroLegacyRos2::topic_callback_cmd(const std_msgs::msg::String & msg)
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
         //this->dl->setup("/dev/ttyUSB0", false);
         this->dl->setup("/dev/ttyUSB0", true);
         this->started = true;
         return;
      }
      else if( s0.compare("close") == 0 )
      {
         RCLCPP_INFO(this->get_logger(), "Darwin-Lofaro Legacy: Close");
         this->started = false;
         this->dl->close();
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
             this->dl->setup("/dev/ttyUSB0");
             this->dl->on();
             */
             printf("Setting up Darwin-Lofaro Legacy\n");
             this->dl->setup("/dev/ttyUSB0", true);
             this->dl->sleep(2.0);
             printf("Turning on actuators\n");
             this->dl->on();
             this->dl->sleep(2.0);
             printf("Actuators On\n");
             this->started = true;


             if(length < 3)
             {
                /* Default timer will be 100hz */
                this->mode = RATE_100HZ;
                this->timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
               return;
             }
             else
             {
               std::string s2 = dout[2];
               if( s2.compare("100") == 0 )
               { 
                 this->mode = RATE_100HZ;
                 this->timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
                 return;
               }
               else if( s2.compare("125") == 0 )
               { 
                 this->mode = RATE_125HZ;
                 this->timer_ = this->create_wall_timer(8ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
                 return;
               }
               else if( s2.compare("50") == 0 )
               { 
                 this->mode = RATE_50HZ;
                 this->timer_ = this->create_wall_timer(20ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
                 return;
               }
               else if( s2.compare("100-motor-state") == 0 )
               { 
                 this->mode = RATE_100HZ_MOTOR_STATE;
                 this->timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
                 return;
               }
                
             }

             this->mode = RATE_100HZ;
             this->timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));
             return;


           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing on Darwin-Lofaro Legacy Mot: %d", mot_num);
                this->dl->on(mot_num);
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
             this->started = false;
             this->dl->off();
             this->dl->stop();
           }
           else
           {
              try
              {
                int mot_num = std::stoi(s1);
                RCLCPP_INFO(this->get_logger(), "Turing off Darwin-Lofaro Legacy Mot: %d", mot_num);
                this->dl->off(mot_num);
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





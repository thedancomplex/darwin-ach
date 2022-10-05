void Ros2AchBridge::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
      if(this->do_debug) RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

      std::string str_msg = msg.data;

      const char delim = ' ';
      std::vector<std::string> dout;
      lofaro_utils_ros2::do_split(str_msg, delim, dout);

      int i = 0;
      int length = dout.size();

      if( length     <  3 ) return;
      if( (length%2) != 1 ) return;
      
      std::string s_head = dout[0];
      double scale = 1.0;
      if     ( s_head.compare("post") == 0 )
      {
	 this->dac.postRef();
	 return;
      }
      else if( s_head.compare("d") == 0 )
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
        try
        {
         std::string s0 = dout[i];
         std::string s1 = dout[i+1];
         int    mot_num = std::stoi(s0);
         double mot_val = std::stod(s1) * scale;

         if(mot_val >  (M_PI / 2.0)) return;
         if(mot_val < -(M_PI / 2.0)) return;
         this->dac.stageRefPos(mot_num,mot_val);
         throw 1;
        }
        catch(...){}
      }

      return;
}


using namespace std::chrono_literals;

#define ENUM_FT_LEFT                       0
#define ENUM_FT_RIGHT                      1


#define DARWIN_TOPIC_REF_POS         "/darwin/ref/position"
#define DARWIN_TOPIC_REF_VEL         "/darwin/ref/speed"
#define DARWIN_TOPIC_REF_TOR         "/darwin/ref/torque"
#define DARWIN_TOPIC_STATE_IMU       "/darwin/state/imu"
#define DARWIN_TOPIC_STATE_FT_LEFT   "/darwin/state/ft/left"
#define DARWIN_TOPIC_STATE_FT_RIGHT  "/darwin/state/ft/right"

DarwinLofaroLegacyRos2::DarwinLofaroLegacyRos2() : Node("darwin_lofaro_legacy_daemon")
{
  subscription_ref_pos_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_pos, this, _1));

  subscription_ref_vel_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_vel, this, _1));

  subscription_ref_tor_ = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR, 10, std::bind(&DarwinLofaroLegacyRos2::topic_callback_ref_tor, this, _1));

publisher_state_imu_ = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_IMU, 1);

publisher_state_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT, 1);

publisher_state_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT, 1);

timer_ = this->create_wall_timer(10ms, std::bind(&DarwinLofaroLegacyRos2::timer_callback_main_loop, this));


}


void DarwinLofaroLegacyRos2::timer_callback_main_loop()
{
  auto buff_imu      = geometry_msgs::msg::Twist();
  auto buff_ft_left  = geometry_msgs::msg::Twist();
  auto buff_ft_right = geometry_msgs::msg::Twist();
 

  this->dl->getImu();
  this->dl->getFt();

  buff_imu.linear.x  = this->dl->darwin_data.imu.acc_x;
  buff_imu.linear.y  = this->dl->darwin_data.imu.acc_y;
  buff_imu.linear.z  = this->dl->darwin_data.imu.acc_z;
  buff_imu.angular.x = this->dl->darwin_data.imu.gyro_x;
  buff_imu.angular.y = this->dl->darwin_data.imu.gyro_y;
  buff_imu.angular.z = this->dl->darwin_data.imu.gyro_z;

  int id = ENUM_FT_LEFT;
  buff_ft_left.linear.x = this->dl->darwin_data.ft[id].x;
  buff_ft_left.linear.y = this->dl->darwin_data.ft[id].y;
  buff_ft_left.linear.z = ( (this->dl->darwin_data.ft[id].raised_x) | 
                             (this->dl->darwin_data.ft[id].raised_y) );
  buff_ft_left.angular.x = this->dl->darwin_data.ft[id].raised_x ;
  buff_ft_left.angular.y = this->dl->darwin_data.ft[id].raised_y;


  id = ENUM_FT_RIGHT;
  buff_ft_right.linear.x = this->dl->darwin_data.ft[id].x;
  buff_ft_right.linear.y = this->dl->darwin_data.ft[id].y;
  buff_ft_right.linear.z = ( (this->dl->darwin_data.ft[id].raised_x) | 
                             (this->dl->darwin_data.ft[id].raised_y) );
  buff_ft_right.angular.x = this->dl->darwin_data.ft[id].raised_x ;
  buff_ft_right.angular.y = this->dl->darwin_data.ft[id].raised_y;

  publisher_state_imu_->publish(buff_imu);
  publisher_state_ft_left_->publish(buff_ft_left);
  publisher_state_ft_right_->publish(buff_ft_right);
}




void DarwinLofaroLegacyRos2::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

void DarwinLofaroLegacyRos2::topic_callback_ref_vel(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

void DarwinLofaroLegacyRos2::topic_callback_ref_tor(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

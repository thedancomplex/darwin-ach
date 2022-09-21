using namespace std::chrono_literals;

DarwinLofaroLegacyRos2Examples::DarwinLofaroLegacyRos2Examples() : Node("darwin_lofaro_interface")
{ 
  /* Zero out the data */
  memset(&this->darwin_data, 0, sizeof(this->darwin_data));

  subscription_state_imu_       = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_IMU,     
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_imu_,       this, _1));
  subscription_state_ft_left_   = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT, 
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_left_,   this, _1));
  subscription_state_ft_right_  = this->create_subscription<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_right_,  this, _1));
  subscription_state_motor_pos_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_POS,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_pos_, this, _1));
  subscription_state_motor_vel_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VEL,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vel_, this, _1));
  subscription_state_motor_tor_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TOR,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tor_, this, _1));
  subscription_state_motor_vol_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VOL,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vol_, this, _1));
  subscription_state_motor_tmp_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TMP,
                                  1, std::bind(&DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tmp_, this, _1));

  publisher_ref_pos_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS, 1);
  publisher_ref_vel_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL, 1);
  publisher_ref_tor_ = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR, 1);
  publisher_cmd_     = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_CMD,     1);
  publisher_clock_   = this->create_publisher<std_msgs::msg::String>(DARWIN_TOPIC_CLOCK,   1);

}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_imu_(const geometry_msgs::msg::Twist & msg)
{
  this->darwin_data.imu.acc_x = msg.linear.x;
  this->darwin_data.imu.acc_y = msg.linear.y;
  this->darwin_data.imu.acc_z = msg.linear.z;
  this->darwin_data.imu.gyro_x = msg.angular.x;
  this->darwin_data.imu.gyro_y = msg.angular.y;
  this->darwin_data.imu.gyro_z = msg.angular.z;
  printf("acc_x = %f\n", this->darwin_data.imu.acc_x);
  return;
}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_left_(const geometry_msgs::msg::Twist & msg)
{
  this->darwin_data.ft[ENUM_FT_LEFT].x        = msg.linear.x;
  this->darwin_data.ft[ENUM_FT_LEFT].y        = msg.linear.y;
  this->darwin_data.ft[ENUM_FT_LEFT].raised   = (int16_t)msg.linear.z;
  this->darwin_data.ft[ENUM_FT_LEFT].raised_x = (int16_t)msg.angular.x;
  this->darwin_data.ft[ENUM_FT_LEFT].raised_y = (int16_t)msg.angular.y;
  return;
}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_ft_right_(const geometry_msgs::msg::Twist & msg)
{
  this->darwin_data.ft[ENUM_FT_RIGHT].x        = msg.linear.x;
  this->darwin_data.ft[ENUM_FT_RIGHT].y        = msg.linear.y;
  this->darwin_data.ft[ENUM_FT_RIGHT].raised   = (int16_t)msg.linear.z;
  this->darwin_data.ft[ENUM_FT_RIGHT].raised_x = (int16_t)msg.angular.x;
  this->darwin_data.ft[ENUM_FT_RIGHT].raised_y = (int16_t)msg.angular.y;
  return;
}

void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_pos_(const std_msgs::msg::Float64MultiArray & msg)
{
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++)
  {
    this->darwin_data.motor_state[i].pos = msg.data[i];
  }
  return;
}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vel_(const std_msgs::msg::Float64MultiArray & msg)
{
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++)
  {
    this->darwin_data.motor_state[i].speed = msg.data[i];
  }
  return;
}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tor_(const std_msgs::msg::Float64MultiArray & msg)
{
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++)
  {
    this->darwin_data.motor_state[i].load = msg.data[i];
  }
  return;
}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_vol_(const std_msgs::msg::Float64MultiArray & msg)
{
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++)
  {
    this->darwin_data.motor_state[i].voltage = msg.data[i];
  }
  return;
}
void DarwinLofaroLegacyRos2Examples::topic_callback_state_motor_tmp_(const std_msgs::msg::Float64MultiArray & msg)
{
  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++)
  {
    this->darwin_data.motor_state[i].temp = msg.data[i];
  }
  return;
}

void DarwinLofaroLegacyRos2Examples::set_motor_pos(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return;
  this->darwin_data.motor_ref[mot].pos = val;
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_pos_->publish(buff);
  return;
}

void DarwinLofaroLegacyRos2Examples::set_motor_vel(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return;
  this->darwin_data.motor_ref[mot].speed = val;
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_vel_->publish(buff);
  return;
}

void DarwinLofaroLegacyRos2Examples::set_motor_tor(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return;
  this->darwin_data.motor_ref[mot].torque = val;
  auto buff = std_msgs::msg::String();
  buff.data = std::to_string(mot) + " " + std::to_string(val);
  publisher_ref_tor_->publish(buff);
  return;
}


using namespace std::chrono_literals;

#include "lofaro_utils_ros2.h"
#include "ros2_ach_bridge_methods_ref_pos.h"
#include "ros2_ach_bridge_methods_ref_vel.h"
#include "ros2_ach_bridge_methods_ref_tor.h"
#include "ros2_ach_bridge_methods_cmd.h"


Ros2AchBridge::Ros2AchBridge(int mode) : Node("darwin_lofaro_legacy_daemon")
{

//  this->dl = (DarwinLofaro*)malloc(sizeof(DarwinLofaro));
//  this->dl = new DarwinLofaro();

  if(mode == MODE_BRIDGE_REF)
  {

    for( int i = DARWIN_MOT_MIN; i <= DARWIN_MOT_MAX; i++)
    {
      this->dac.stageRefPos(   i, DARWIN_REF_POS_0);
      this->dac.stageRefVel(   i, DARWIN_REF_VEL_0);
      this->dac.stageRefTorque(i, DARWIN_REF_TOR_0);
    }
    this->dac.postRef();

    subscription_ref_pos_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_POS,         10, std::bind(&Ros2AchBridge::topic_callback_ref_pos, this, _1));
    subscription_ref_vel_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_VEL,         10, std::bind(&Ros2AchBridge::topic_callback_ref_vel, this, _1));
    subscription_ref_tor_     = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_REF_TOR,         10, std::bind(&Ros2AchBridge::topic_callback_ref_tor, this, _1));
    subscription_cmd_         = this->create_subscription<std_msgs::msg::String>(DARWIN_TOPIC_CMD,             10, std::bind(&Ros2AchBridge::topic_callback_cmd, this, _1));
  }
  else if(mode == MODE_BRIDGE_STATE )
  {
    publisher_state_ft_left_  = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_LEFT,  1);
    publisher_state_ft_right_ = this->create_publisher<geometry_msgs::msg::Twist>(DARWIN_TOPIC_STATE_FT_RIGHT, 1);
    publisher_state_motor_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_POS, 1);
    publisher_state_motor_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VEL, 1);
    publisher_state_motor_tor_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TOR, 1);
    publisher_state_motor_vol_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_VOL, 1);
    publisher_state_motor_tmp_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(DARWIN_TOPIC_STATE_MOTOR_TMP, 1);
  }
}

int Ros2AchBridge::state_loop()
{
  this->dac.getState(true);
  auto buff_imu       = geometry_msgs::msg::Twist();
  auto buff_ft_left   = geometry_msgs::msg::Twist();
  auto buff_ft_right  = geometry_msgs::msg::Twist();
  auto buff_motor_pos = std_msgs::msg::Float64MultiArray();
  auto buff_motor_vel = std_msgs::msg::Float64MultiArray();
  auto buff_motor_tor = std_msgs::msg::Float64MultiArray();
  auto buff_motor_vol = std_msgs::msg::Float64MultiArray();
  auto buff_motor_tmp = std_msgs::msg::Float64MultiArray();

  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
  {
    buff_motor_pos.data.push_back(this->dac.darwin_data.motor_state[i].pos);
    buff_motor_vel.data.push_back(this->dac.darwin_data.motor_state[i].speed);
    buff_motor_tor.data.push_back(this->dac.darwin_data.motor_state[i].load);
    buff_motor_vol.data.push_back(this->dac.darwin_data.motor_state[i].voltage);
    buff_motor_tmp.data.push_back(this->dac.darwin_data.motor_state[i].temp);
  }

  buff_imu.linear.x  =         this->dac.darwin_data.imu.acc_x;
  buff_imu.linear.y  =         this->dac.darwin_data.imu.acc_y;
  buff_imu.linear.z  =         this->dac.darwin_data.imu.acc_z;
  buff_imu.angular.x =         this->dac.darwin_data.imu.gyro_x;
  buff_imu.angular.y =         this->dac.darwin_data.imu.gyro_y;
  buff_imu.angular.z =         this->dac.darwin_data.imu.gyro_z;

  int id = ENUM_FT_LEFT;
  buff_ft_left.linear.x  =     this->dac.darwin_data.ft[id].x;
  buff_ft_left.linear.y  =     this->dac.darwin_data.ft[id].y;
  buff_ft_left.linear.z  = (  (this->dac.darwin_data.ft[id].raised_x) | (this->dac.darwin_data.ft[id].raised_y)  );
  buff_ft_left.angular.x =     this->dac.darwin_data.ft[id].raised_x ;
  buff_ft_left.angular.y =     this->dac.darwin_data.ft[id].raised_y;


  id = ENUM_FT_RIGHT;
  buff_ft_right.linear.x  =    this->dac.darwin_data.ft[id].x;
  buff_ft_right.linear.y  =    this->dac.darwin_data.ft[id].y;
  buff_ft_right.linear.z  = ( (this->dac.darwin_data.ft[id].raised_x) | (this->dac.darwin_data.ft[id].raised_y) );
  buff_ft_right.angular.x =    this->dac.darwin_data.ft[id].raised_x ;
  buff_ft_right.angular.y =    this->dac.darwin_data.ft[id].raised_y;

  publisher_state_imu_->publish(buff_imu);
  publisher_state_ft_left_->publish(buff_ft_left);
  publisher_state_ft_right_->publish(buff_ft_right);
  publisher_state_motor_pos_->publish(buff_motor_pos);
  publisher_state_motor_vel_->publish(buff_motor_vel);
  publisher_state_motor_tor_->publish(buff_motor_tor);
  publisher_state_motor_vol_->publish(buff_motor_vol);
  publisher_state_motor_tmp_->publish(buff_motor_tmp);

  return 0;
}

/*
void Ros2AchBridge::topic_callback_ref_pos(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_ref_vel(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_ref_tor(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

/*
void Ros2AchBridge::topic_callback_cmd(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
*/

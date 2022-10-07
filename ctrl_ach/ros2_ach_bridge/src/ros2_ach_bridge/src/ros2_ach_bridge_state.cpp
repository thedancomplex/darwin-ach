#include <ros2_ach_bridge.h>

#define LOOP_TIMEOUT 30
#define LOOP_TIMEOUT_SLEEP_PERIOD 0.02
#define LOOP_ENABLE_SECONDS 5.0
#define LOOP_ENABLE_SECONDS_COUNT LOOP_ENABLE_SECONDS / LOOP_TIMEOUT_SLEEP_PERIOD


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  Ros2AchBridge rab = Ros2AchBridge(MODE_BRIDGE_STATE);

  double tick_timeout = rab.dac.time();

  int r = 0;

  while(true)
  {
    double tock_timeout = rab.dac.time();
    double dt = tock_timeout - tick_timeout;
    if( dt > LOOP_TIMEOUT )
    {
      printf("1\n");
      return 1;    
    }

    int d = rab.dac.getTime();
    if ( d == 0 ) r++;
    if( r > LOOP_ENABLE_SECONDS_COUNT ) break;

    rab.dac.sleep(LOOP_TIMEOUT_SLEEP_PERIOD);
  }

  printf("Start Ach2Ros State Loop\n");

  while(true)
  {
    rab.state_loop();
  }
  rclcpp::shutdown();
  printf("0\n");
  return 0;
}


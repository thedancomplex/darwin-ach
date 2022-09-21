#include <darwin_lofaro_examples.h>
#include "lofaro_utils.h"
std::thread::id darwin_spin_thread_id;

pthread_t thread1;

void do_spin(int val)
{
  darwin_spin_thread_id = std::this_thread::get_id();
  rclcpp::spin(std::make_shared<DarwinLofaroLegacyRos2Examples>());
}

int start()
{
printf("a\n");
  int ret = 0;
printf("b\n");
  int argc = 1;
printf("c\n");
  char *argv[1];
printf("d\n");
  argv[0] = "darwin_lofaro_examples";
printf("e\n");
  rclcpp::init(argc, argv);

printf("f\n");
  //ret = pthread_create(&thread1, NULL, do_spin, 0);
  std::thread do_spin_thread(do_spin, 0);
printf("g\n");
  do_spin_thread.detach();
printf("h\n");
  darwin_spin_thread_id = do_spin_thread.get_id();
printf("i\n");
  return ret;
}

int stop()
{
  int ret = 0;

  pthread_t thread_id = std::hash<std::thread::id>{}(darwin_spin_thread_id);


  //ret += thread1.request_stop();
  ret += pthread_cancel( thread_id );
  ret += rclcpp::shutdown();
  if (ret > 0) ret = 1;
  return ret;
}


int main(int argc, char * argv[])
{

  LofaroUtils* lut = new LofaroUtils();

printf("1\n");
  start();
printf("2\n");
  lut->sleep(60.0);
printf("3\n");
  stop();
printf("4\n");
  return 0;
}


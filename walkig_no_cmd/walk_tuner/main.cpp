#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <signal.h>
#include <libgen.h>
#include "cmd_process.h"
#include "mjpg_streamer.h"

#define INI_FILE_PATH       "Data/config.ini"

using namespace Robot;

LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

int main(int argc, char *argv[])
{
  printf("1\n");
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

  printf("2\n");
    change_current_dir();

  printf("3\n");
    minIni* ini = new minIni(INI_FILE_PATH);

  printf("4\n");
//    mjpg_streamer* streamer = new mjpg_streamer(0, 0);
//    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        return 0;
    }
  printf("5\n");
    MotionManager::GetInstance()->LoadINISettings(ini);
  printf("6\n");
    Walking::GetInstance()->LoadINISettings(ini);

  printf("7\n");
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
  printf("8\n");
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  printf("9\n");
    motion_timer->Start();
  printf("10\n");
    /////////////////////////////////////////////////////////////////////

//    DrawIntro(&cm730);
  printf("11\n");
    MotionManager::GetInstance()->SetEnable(true);
  printf("12\n");


  printf("Walking Start for 10 seconds\n");
    MotionManager::GetInstance()->StartLogging();
    Walking::GetInstance()->Start();

    sleep(10);

  printf("Stop Walking\n");
    Walking::GetInstance()->Stop();
    MotionManager::GetInstance()->StopLogging();

    sleep(2);

//    DrawEnding();
    return 0;
}

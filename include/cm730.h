#if !defined(DYN_CM730)
#define DYN_CM730 1
#endif

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdio.h>
#include <dynamixel_sdk.h>                                   // Uses Dynamixel SDK library
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
int DXL_ID = 200;                   // Dynamixel ID: 200 - cm730
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller



// Enumbs and defines
#define CM730_ON  1
#define CM730_OFF 0

// Addresses 
#define CM730_ADDRESS_DYN_POWER 24

namespace darwin {

  int close();
  int open();
  int getch();
  int on(int val);
  int kbhit(void);
  int pint(int val);
  

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  int close()
  {
    portHandler->closePort();
    return 0;
  }

  int open()
  {
    // Open port
    if (portHandler->openPort())
    {
      printf("Succeeded to open the port!\n");
    }
    else
    {
      printf("Failed to open the port!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 1;
    }
    return 0;
  }


  int getch()
  {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }

  int kbhit(void)
  {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }

    return 0;
  }




  int on(int val)
  {

  }

  int ping(int val)
  {
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_model_number;                      // Dynamixel model number

    dxl_comm_result = packetHandler->ping(portHandler, val, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return 1;
    }
    else {
      printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", val, dxl_model_number);
    }
    return 0;
  }



}

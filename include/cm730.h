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
#define DARWIN_ON  CM730_ON
#define DARWIN_OFF CM730_OFF

// Motor IDs
#define ID_CM730 200
#define ID_DARWIN ID_CM730

// Addresses 
#define CM730_ADDRESS_DYN_POWER 24
#define CM730_ADDRESS_IMU_GYRO_Z 38
#define CM730_ADDRESS_IMU_GYRO_Y 40
#define CM730_ADDRESS_IMU_GYRO_X 42
#define CM730_ADDRESS_IMU_ACC_X 44
#define CM730_ADDRESS_IMU_ACC_Y 46
#define CM730_ADDRESS_IMU_ACC_Z 48

namespace darwin {

  int close();
  int open();
  int getch();
  int on(int val);
  int off(int val);
  int kbhit(void);
  int ping(int val);
  int update_imu();
  double int2double(uint16_t val); 

  // IMU data
  double imu_gyro_x = 0.0; 
  double imu_gyro_y = 0.0; 
  double imu_gyro_z = 0.0; 
  double imu_acc_x  = 0.0; 
  double imu_acc_y  = 0.0; 
  double imu_acc_z  = 0.0; 

  // Voltage
  double voltage = 0.0;


  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  double int2double(uint16_t val)
  {
    double the_out = (double)((int32_t)val - 512) / 1023.0;
    return the_out;
  }

  int update_imu()
  {
    // Read IMU info
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    int the_imu_val = CM730_ADDRESS_IMU_GYRO_X;
    uint16_t imu_tmp = 0;

    int imu_i[] = {CM730_ADDRESS_IMU_GYRO_Z, CM730_ADDRESS_IMU_GYRO_Y, CM730_ADDRESS_IMU_GYRO_X,
                   CM730_ADDRESS_IMU_ACC_X,  CM730_ADDRESS_IMU_ACC_Y,  CM730_ADDRESS_IMU_ACC_Z};
    int imu_max = sizeof(imu_i) / sizeof(imu_i[0]);
    int ret = 0;
    for( int i = 0; i < imu_max; i++ )
    {
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID_CM730, the_imu_val, &imu_tmp, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        if     ( i == CM730_ADDRESS_IMU_GYRO_Z ) imu_gyro_z = int2double(imu_tmp) * 500.0; // +-500 deg/sec
        else if( i == CM730_ADDRESS_IMU_GYRO_Y ) imu_gyro_y = int2double(imu_tmp) * 500.0; // +-500 deg/sec
        else if( i == CM730_ADDRESS_IMU_GYRO_X ) imu_gyro_x = int2double(imu_tmp) * 500.0; // +- 500 deg/sec
        else if( i == CM730_ADDRESS_IMU_ACC_X  ) imu_acc_x  = int2double(imu_tmp) * 4.0; // +-4g
        else if( i == CM730_ADDRESS_IMU_ACC_Y  ) imu_acc_y  = int2double(imu_tmp) * 4.0; // +-4g
        else if( i == CM730_ADDRESS_IMU_ACC_Z  ) imu_acc_z  = int2double(imu_tmp) * 4.0; // +-4g
        else ret = 1;
      }
      else ret = 1;
    }


    return ret;
  }


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



  int off(int val)
  {
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, val, CM730_ADDRESS_DYN_POWER, CM730_OFF, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return 1;
    }
    else
    {
      printf("Dynamixel has been successfully turned off \n");
    }
    return 0;
  }

  int on(int val)
  {
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, val, CM730_ADDRESS_DYN_POWER, CM730_ON, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return 1;
    }
    else
    {
      printf("Dynamixel has been successfully turned on \n");
    }
    return 0;
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

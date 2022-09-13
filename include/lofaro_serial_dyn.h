// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdint>

#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"


namespace lofaro {
#include "lofaro_utils.h"

// Protocol version
#define PROTOCOL_VERSION                1.0  
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
#define ESC_ASCII_VALUE                 0x1b
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque


#define SERIAL_PKT_LENGTH 3

#define DYN_ID_ALL        0xfe

#define DYN_PING          0x01
#define DYN_READ          0x02
#define DYN_WRITE         0x03
#define DYN_WRITE_REG     0x04
#define DYN_ACTION        0x05
#define DYN_FACTORY_RESET 0x06
#define DYN_REBOOT        0x08 
#define DYN_WRITE_SYNC    0x83
#define DYN_READ_BULK     0x92

#define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"

int serial_port = 0;

uint8_t   get_checksum(uint8_t *rxpacket);
int       do_open();
int       do_open(const char* the_serial_port);
int       do_close();
int       do_write(uint8_t id, uint8_t address, uint8_t d0);
int       do_write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1);
int       do_write(uint8_t id, uint8_t address, uint8_t length, uint8_t *dn);
int       do_write_set(uint8_t id, uint8_t address, uint8_t d0);
int       do_write_set(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1);
int       do_write_send();
int       do_write_send(uint8_t id);
int       do_read(uint8_t id, uint8_t address);
int       do_read(uint8_t id, uint8_t address, uint8_t length);
int       do_read(uint8_t id, uint8_t buff[], uint8_t buff_length);
int       do_read_buffer(uint8_t buff[1024], int *the_length);
int       check_serial();
void      do_flush();
int       do_flush_final();
int getch();
int kbhit(void)

struct termios tty;


// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler;

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler;



bool PORT_STATUS = false;

int check_serial()
{
  if(PORT_STATUS) return 0;
  return 0;
}

int do_open()
{
  return do_open(SERIAL_PORT_DEFAULT);
}

int do_open(const char* the_serial_port)
{

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    *portHandler = dynamixel::PortHandler::getPortHandler(the_serial_port);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->setPacketTimeout(0.001);
    // Open port
    if (portHandler->openPort())
    {
      PORT_STATUS = true;
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


int do_flush_final()
{
  return 0;
}

int do_close()
{
  // Close port
  portHandler->closePort();
  PORT_STATUS = false;
  return 0;
}

int do_write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
{
    int buff[] = {d0, d1};
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, address, buff, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

  return 0;
}

int do_write_set(uint8_t id, uint8_t address, uint8_t d0)
{
  return 0;
}

int do_write(uint8_t id, uint8_t address, uint8_t d0)
{
    uint8_t val = d0;

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, address, val, &dxl_error);
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

int do_write_send()
{
  return do_write_send( DYN_ID_ALL );
}

int do_write_send(uint8_t id)
{
  return 0;
}

void do_flush()
{
  return;
}

int do_read(uint8_t id, uint8_t buff[], uint8_t buff_length)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead gb = new groupBulkRead(portHandler, packetHandler);
  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = gb.groupBulkRead.addParam(id, address, length_read);

  // Bulkread present position and moving status
  dxl_comm_result = gb.groupBulkRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (groupBulkRead.getError(id, &dxl_error))
  {
    printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
  }

  dxl_getdata_result = groupBulkRead.isAvailable(id, address, length_read);
  if (dxl_getdata_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
    return 0;
  }
    // Get Dynamixel#1 present position value
    buff = gb.groupBulkRead.getData(id, address, buff_length);

  /* this is the bulk read */
  return 0;
}

int do_read(uint8_t id, uint8_t address, uint8_t length_read)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint16_t buff_out_2byte = 0;
  uint8_t  buff_out_1byte = 0;

  if(length_read == 2)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &buff_out_2byte, &dxl_error);
  }
  else if(length_read == 1)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &buff_out_1byte, &dxl_error);
  }
  else 
  {
    return 1;
  }
  return 0;
}

int do_read(uint8_t id, uint8_t address)
{
  return do_read(id, address, 1);
}


}

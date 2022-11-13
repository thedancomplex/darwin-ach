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

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupBulkRead instance
dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

namespace lofaro {
#include "lofaro_utils.h"

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
#define ESC_ASCII_VALUE                 0x1b
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

// Buffer positions
#define BUFFER_ID 2
#define BUFFER_LENGTH 3 

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
int       getch();
int       kbhit(void);

uint8_t   rx_buff[1024];

bool      PORT_STATUS = false;

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
    memset(&rx_buff, 0, sizeof(rx_buff));

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
    uint16_t buff = 0;
    buff = buff & d1;
    buff = (buff << 8) & d0;
    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

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

int do_write_set(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
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
      printf("Success \n");
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
  /* this is the bulk read */
  return 0;
}

int do_read(uint8_t id, uint8_t address, uint8_t length_read)
{
  bool dxl_addparam_result = false;               // addParam result
  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint16_t buff_out_2byte = 0;
  uint16_t buff_out_1byte = 0;

  if( length_read <= 0 ) return 1;

  // clear paramaters
  groupBulkRead.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupBulkRead.addParam(id, address, length_read);

  // Bulkread present position and moving status
  dxl_comm_result = groupBulkRead.txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (groupBulkRead.getError(id, &dxl_error))
  {
    printf("[ID:%03d] %s\n", id, packetHandler->getRxPacketError(dxl_error));
  }
  dxl_getdata_result = groupBulkRead.isAvailable(id, address, length_read);
  if (dxl_getdata_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
    return 1;
  }

  memset(&rx_buff, 0, sizeof(rx_buff));
  rx_buff[0] = 0xff;
  rx_buff[1] = 0xff;
  rx_buff[2] = id;
  rx_buff[3] = length_read;
  rx_buff[4] = 0x00;
  int current_i = 5;

  for (int i = 0; i < length_read; i++)
  {
    rx_buff[current_i+i] = (uint8_t)groupBulkRead.getData(id, address+i, 1);
  }
/*
  for (int i = 0; i < (length_read + 4); i = i+4)
  {
    int read_length_2 = length_read - i;
    int address_2 = address + i;
    uint32_t buff = 0;

    // Get Dynamixel#1 present position value
    buff = groupBulkRead.getData(id, address_2, read_length_2);
    int io = current_i + i;
    uint8_t b0 = (uint8_t)((buff << 0 ) & 0xff);
    uint8_t b1 = (uint8_t)((buff << 8 ) & 0xff);
    uint8_t b2 = (uint8_t)((buff << 16) & 0xff);
    uint8_t b3 = (uint8_t)((buff << 24) & 0xff);
    
    rx_buff[io+0] = b0;
    rx_buff[io+1] = b1;
    rx_buff[io+2] = b2;
    rx_buff[io+3] = b3;
  }
*/

  uint8_t the_checksum = get_checksum(rx_buff);

  rx_buff[length_read + 3] = the_checksum;

  return 0;
}

int do_read(uint8_t id, uint8_t address)
{
  return do_read(id, address, 1);
}

uint8_t get_checksum(uint8_t *rxpacket)
{
  uint8_t checksum    = 0;
  uint8_t wait_length = rxpacket[SERIAL_PKT_LENGTH] + SERIAL_PKT_LENGTH + 1;


  // calculate checksum
  for (uint16_t i = 2; i < wait_length - 1; i++)   // except header, checksum
  {
    checksum += rxpacket[i];
  }
  checksum = ~checksum;
  return checksum;
}


int check_checksum( uint8_t buff[] )
{
   uint8_t cs     = get_checksum(buff);
   uint8_t cs_i   = buff[BUFFER_LENGTH] + 2 + 1;
   uint8_t cs_val = buff[cs_i];
   if ( cs == cs_val ) return 0;
   return 1;
}


int do_get_length( uint8_t buff[] )
{
   uint8_t cs_i = buff[BUFFER_LENGTH] + 2 + 1;
   return cs_i;
}

int do_read_buffer(uint8_t buff[1024], int *the_length)
{
//  strncpy(buff, rx_buff, sizeof(rx_buff));
/*
  for(int i = 0; i < 1024; i++)
  {
    buff[i] = rx_buff[i];
  }
*/
  memcpy(buff, rx_buff, sizeof(rx_buff));
  *the_length = do_get_length(buff);
  for(int i = 0; i < *the_length+3; i++) printf("%x ",buff[i]);
  printf("\n");
  for(int i = 0; i < *the_length+3; i++) printf("%x ",rx_buff[i]);
  printf("\n");
  printf("%d\n", *the_length);
  return 0;
}


}

/*******************************************************************************
* Copyright 2022 Daniel M. Lofaro
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Daniel M. Lofaro */

#ifndef DARWIN_DEFINE
#include "lofaro_define.h"
#endif

#ifndef DARWIN_INCLUDE
#include "lofaro_include.h"
#endif




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
//#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#include <sys/ioctl.h>


namespace darwin {

  int close();
  int open();
  int getch();
  int on(int val);
  int off(int val);
  int kbhit(void);
  int ping(int val);
  int update_imu();
  int update_imu_setup();
  int update_imu_slow();
  int update_ft();
  int update_ft_setup();
  double int2double(uint16_t val); 
  double uint2double(uint16_t val); 
  double ft_char2double(uint8_t val, int* err);
  uint8_t read1byte(uint8_t id, uint8_t address);
  int write1byte(uint8_t id, uint8_t address, uint8_t val);
  int flush();

  // IMU data
  double imu_gyro_x = -0.0; 
  double imu_gyro_y = -0.0; 
  double imu_gyro_z = -0.0; 
  double imu_acc_x  = -0.0; 
  double imu_acc_y  = -0.0; 
  double imu_acc_z  = -0.0; 

  // FT data
  double ft_left_x  = -0.0;
  double ft_left_y  = -0.0;
  double ft_right_x = -0.0;
  double ft_right_y = -0.0;
  double ft_fsr_x   = -0.0;
  double ft_fsr_y   = -0.0;
  int ft_fsr_raised_x = RAISED;
  int ft_fsr_raised_y = RAISED;


  // Voltage
  double voltage = -0.0;
  double voltage_foot = -0.0;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkReadImu(portHandler, packetHandler);
  dynamixel::GroupBulkRead groupBulkReadFt(portHandler, packetHandler);

  int flush()
  {
    portHandler->clearPort();
    //portHandler->flush();
    return 0;
  }

  double int2double(uint16_t val)
  {
    double the_out = (double)((int32_t)val - 512) / 1023.0;
    return the_out;
  }

  double uint2double(uint16_t val)
  {
    return (double)(val) / 65535.0;
  }

  int get_ft()
  {
    groupBulkReadFt.clearParam();
    // Add parameter storage for Dynamixel#1 present position value
    // +1 is added to read the voltage
    dxl_addparam_result = groupBulkReadFt.addParam(ID_FT, FT_ADDRESS_START, FT_ADDRESS_LENGTH);

    bool dxl_getdata_result = false;                // GetParam result
    uint8_t dxl_error = 0;                          // Dynamixel error

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    dxl_comm_result = groupBulkReadFt.txRxPacket();
    packetHandler->getTxRxResult(dxl_comm_result);
    if (groupBulkReadFt.getError(ID_FT, &dxl_error)) return 1;

    // Check if data is avaliable
    dxl_getdata_result = groupBulkReadFt.isAvailable(ID_FT, FT_ADDRESS_START, FT_ADDRESS_LENGTH);
    if (dxl_getdata_result != true) return 1;

    // Assign the data
    uint16_t buff_left_x   = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_S1, 2);
    uint16_t buff_left_y   = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_S2, 2);
    uint16_t buff_right_x  = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_S3, 2);
    uint16_t buff_right_y  = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_S4, 2);
    uint16_t buff_fsr_x    = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_FSR_X, 2);
    uint16_t buff_fsr_y    = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_FSR_Y, 2);

    ft_left_x   = int2double(buff_left_x)  * FT_SCALE;
    ft_left_y   = int2double(buff_left_y)  * FT_SCALE;
    ft_right_x  = int2double(buff_right_x) * FT_SCALE;
    ft_right_y  = int2double(buff_right_y) * FT_SCALE;

    ft_fsr_x    = ft_char2double(buff_fsr_x, &ft_fsr_raised_x) * FSR_SCALE_X;
    ft_fsr_y    = ft_char2double(buff_fsr_y, &ft_fsr_raised_y) * FSR_SCALE_Y;

    return 0;
  }

  double ft_char2double(uint8_t val, int* err)
  {
    if( val == 255)
    {
      *err = RAISED;
      return 0.0;
    }

    double the_out = (double)val - 127.0 / 127.0;
    *err = NOT_RAISED;
    return the_out;
  }

  int get_imu()
  {
    groupBulkReadImu.clearParam();
    // Add parameter storage for Dynamixel#1 present position value
    // +1 is added to read the voltage
    dxl_addparam_result = groupBulkReadImu.addParam(ID_CM730, CM730_ADDRESS_IMU_START, CM730_ADDRESS_IMU_LENGTH+1);
    if (dxl_addparam_result != true) return 1;
    bool dxl_getdata_result = false;                // GetParam result
    uint8_t dxl_error = 0;                          // Dynamixel error

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    dxl_comm_result = groupBulkReadImu.txRxPacket();
    packetHandler->getTxRxResult(dxl_comm_result);
    if (groupBulkReadImu.getError(ID_CM730, &dxl_error)) return 1;

    // Check if data is avaliable
    dxl_getdata_result = groupBulkReadImu.isAvailable(ID_CM730, CM730_ADDRESS_IMU_START, CM730_ADDRESS_IMU_LENGTH);
    if (dxl_getdata_result != true) return 1;
    
    // Assign the data
    uint16_t buff_gyro_x = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_X, 2);
    uint16_t buff_gyro_y = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Y, 2);
    uint16_t buff_gyro_z = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Z, 2);
    uint16_t buff_acc_x = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_X, 2);
    uint16_t buff_acc_y = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Y, 2);
    uint16_t buff_acc_z = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Z, 2);
    uint8_t buff_voltage = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_VOLTAGE, 1);

    imu_gyro_x = int2double(buff_gyro_x) * IMU_GYRO_SCALE;
    imu_gyro_y = int2double(buff_gyro_y) * IMU_GYRO_SCALE;
    imu_gyro_z = int2double(buff_gyro_z) * IMU_GYRO_SCALE;
    imu_acc_x  = int2double(buff_acc_x)  * IMU_ACC_SCALE;
    imu_acc_y  = int2double(buff_acc_y)  * IMU_ACC_SCALE;
    imu_acc_z  = int2double(buff_acc_z)  * IMU_ACC_SCALE;
    voltage    = (double)buff_voltage / VOLTAGE_SCALE;

    return 0;
  }

  uint8_t read1byte(uint8_t id, uint8_t address)
  {
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t buff = 0;

    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, address, &buff, &dxl_error);

    if (dxl_error == 0)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
      return buff;
    }

    return 255;
  }


  int close()
  {
    portHandler->closePort();
    return 0;
  }

  int open()
  {
    portHandler->setPacketTimeout(0.01);
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


  int write1byte(uint8_t id, uint8_t address, uint8_t val)
  {
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
      printf("1 Byte Written \n");
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

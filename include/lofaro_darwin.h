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


#if !defined(DYN_CM730)
#define DYN_CM730 1
#endif

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdio.h>

#define BAUDRATE                        1000000
//#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#include <sys/ioctl.h>


#include "lofaro_serial.h"

namespace darwin {


  // Buffer positions
  #define BUFFER_ID 2
  #define BUFFER_LENGTH 3	

  // Enumbs and defines
  #define CM730_ON  1
  #define CM730_OFF 0
  #define DARWIN_ON  CM730_ON
  #define DARWIN_OFF CM730_OFF
  #define IMU_ACC_SCALE 70.67723342939482
  #define IMU_GYRO_SCALE 500.0
  #define VOLTAGE_SCALE 10.0
  #define FT_SCALE 1/1000.0
  #define FSR_SCALE_X 1.0
  #define FSR_SCALE_Y 1.0
  #define RETURN_OK 0
  #define RETURN_FAIL 1

  // Motor IDs
  #define ID_CM730 200
  #define ID_DARWIN ID_CM730
  #define ID_FT 100

  // Addresses 
  #define CM730_ADDRESS_DYN_POWER 24
  #define CM730_ADDRESS_ID 3

  #define CM730_ADDRESS_IMU_START 38
  #define CM730_ADDRESS_IMU_LENGTH 12
  #define CM730_ADDRESS_READ_DATA_OFFSET 5

  #define CM730_ADDRESS_IMU_GYRO_Z 38
  #define CM730_ADDRESS_IMU_GYRO_Y 40
  #define CM730_ADDRESS_IMU_GYRO_X 42
  #define CM730_ADDRESS_IMU_ACC_X 44
  #define CM730_ADDRESS_IMU_ACC_Y 46
  #define CM730_ADDRESS_IMU_ACC_Z 48

  #define CM730_ADDRESS_VOLTAGE 50

  #define FT_ADDRESS_START 26
  #define FT_ADDRESS_LENGTH 10
  #define FT_ADDRESS_LEFT_X 26
  #define FT_ADDRESS_LEFT_Y 28
  #define FT_ADDRESS_RIGHT_X 30
  #define FT_ADDRESS_RIGHT_Y 32
  #define FT_ADDRESS_FSR_X 34
  #define FT_ADDRESS_FSR_Y 35
  #define FT_ADDRESS_VOLTAGE 42


  #define ERROR 1
  #define NO_ERROR 0
  #define RAISED 1
  #define NOT_RAISED 0

  int sleep(double val);
  double time();

  int open();
  int getch();
  int on(int val);
  int off(int val);
  int kbhit(void);
  int ping(int val);
  int update_imu(uint8_t val[]);
  int update_imu_setup();
  int update_imu_slow();
  int update_ft();
  int update_ft_setup();
  double int2double(uint16_t val); 
  double uint2double(uint16_t val); 
  double ft_char2double(uint8_t val, int* err);
  uint8_t read1byte(uint8_t id, uint8_t address);
  int flush();
  uint16_t chars2uInt16(uint8_t d_lsb, uint8_t d_msb);
  int check_head( uint8_t buff[] );
  int check_checksum( uint8_t buff[] );
  int get_next_message( uint8_t buff[], int *the_length );

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


  int read(uint8_t id, uint8_t address);
  int read(uint8_t id, uint8_t address, uint8_t length);
  int read_buffer();

  int sleep(double val)
  {
    long usec = (long)(val * 1000000);
    return usleep(usec);
  }

  int read_buffer()
  {
    uint8_t buff[1024];
    int n = 0;
    int ret = lofaro::do_read_buffer(buff, &n);

    if( RETURN_OK == check_head(buff) )
    {
      uint8_t id = buff[BUFFER_ID];
      if( id == ID_CM730 )
      {
        update_imu(buff);
      }
    }



    while( RETURN_OK == get_next_message(buff, &n) )
    {
      printf("Serial Buff Length = %d\n Buff=\n",n);
      for( int i = 0; i < n; i++ )
      {
        printf("%x ",(uint8_t)buff[i]);
      }
      printf("\n");
      printf("Is head ok %d\n",check_head(buff));
      printf("Is checksum ok %d\n",check_checksum(buff));
    }

    return 0;
  }

  int get_next_message( uint8_t buff[], int *the_length )
  {
    int n = *the_length;
    int ni = n;
    for ( int i = 0; i < ni; i++)
    {
      ni = ni - 1;
      *the_length = ni;
      memcpy(buff, buff + 1*sizeof(buff[0]), (1024-1)*sizeof(buff[0]) );
      if( (RETURN_OK == check_checksum(buff)) & 
          (RETURN_OK == check_head(buff)    )  )
      {
       return RETURN_OK;
      }
    }
    return RETURN_FAIL;
  }

  int check_checksum( uint8_t buff[] )
  {
     uint8_t cs = lofaro::get_checksum(buff);
     uint8_t cs_i = buff[BUFFER_LENGTH] + 2 + 1;
     uint8_t cs_val = buff[cs_i];
     if ( cs == cs_val ) return RETURN_OK;
     return RETURN_FAIL;
  }

  int check_head( uint8_t buff[] )
  {
    if( (buff[0] == 0xff) & (buff[1] == 0xff) ) return 0;
    return 1;
  }
  
  int read(uint8_t id, uint8_t address)
  {
    return lofaro::do_read(id,address);
  }
  
  int read(uint8_t id, uint8_t address, uint8_t length)
  {
    return lofaro::do_read(id,address,length);
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

  int update_ft()
  {
    
    // Assign the data
/*
    uint16_t buff_left_x   = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_LEFT_X, 2);
    uint16_t buff_left_y   = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_LEFT_Y, 2);
    uint16_t buff_right_x  = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_RIGHT_X, 2);
    uint16_t buff_right_y  = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_RIGHT_Y, 2);
    uint16_t buff_fsr_x    = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_FSR_X, 2);
    uint16_t buff_fsr_y    = groupBulkReadFt.getData(ID_FT, FT_ADDRESS_FSR_Y, 2);

    ft_left_x   = int2double(buff_left_x)  * FT_SCALE;
    ft_left_y   = int2double(buff_left_y)  * FT_SCALE;
    ft_right_x  = int2double(buff_right_x) * FT_SCALE;
    ft_right_y  = int2double(buff_right_y) * FT_SCALE;

    ft_fsr_x    = ft_char2double(buff_fsr_x, &ft_fsr_raised_x) * FSR_SCALE_X;
    ft_fsr_y    = ft_char2double(buff_fsr_y, &ft_fsr_raised_y) * FSR_SCALE_Y;
*/

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

  uint16_t chars2uInt16(uint8_t d_lsb, uint8_t d_msb)
  {
    uint16_t d = 0;
    d = d | (uint16_t)d_lsb;
    d = d | ((uint16_t)d_msb << 8);
    return d;
  }

  int update_imu( uint8_t buff[])
  {
// ff ff c8 f 0 0 2 0 2 0  2  5  2  bc 1  79 2  7b 68 0
// 1  2  3  4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19

    if ( RETURN_OK != check_head(buff) ) return RETURN_FAIL;
    if ( RETURN_OK != check_checksum(buff) ) return RETURN_FAIL;
    int length = 19;

    // Assign the diata
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_X) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_X) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_gyro_x = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_Y) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_Y) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_gyro_y = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_Z) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_GYRO_Z) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_gyro_z = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_X) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_X) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_acc_x = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_Y) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_Y) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_acc_y = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_Z) - CM730_ADDRESS_IMU_START;
    b1 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_IMU_ACC_Z) - CM730_ADDRESS_IMU_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_acc_z = chars2uInt16(b0, b1);

    b0 = (CM730_ADDRESS_READ_DATA_OFFSET + CM730_ADDRESS_VOLTAGE) - CM730_ADDRESS_IMU_START;
    b0 = buff[b0];
    uint8_t buff_voltage = b0;

    imu_gyro_x = int2double(buff_gyro_x) * IMU_GYRO_SCALE;
    imu_gyro_y = int2double(buff_gyro_y) * IMU_GYRO_SCALE;
    imu_gyro_z = int2double(buff_gyro_z) * IMU_GYRO_SCALE;
    imu_acc_x  = int2double(buff_acc_x)  * IMU_ACC_SCALE;
    imu_acc_y  = int2double(buff_acc_y)  * IMU_ACC_SCALE;
    imu_acc_z  = int2double(buff_acc_z)  * IMU_ACC_SCALE;
    voltage    = (double)buff_voltage / VOLTAGE_SCALE;

    return 0;
  }

  int close()
  {
    return lofaro::do_close();
  }

  int open()
  {
    return lofaro::do_open();
  }

  int off(int val)
  {
    return 0;
  }

  int on(int val)
  {
    return 0;
  }

  int ping(int val)
  {
    return 0;
  }

  double time()
  {
    return lofaro::get_time();
  }
}

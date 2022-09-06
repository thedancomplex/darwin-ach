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

  # define M_PI           3.14159265358979323846

  // Buffer positions
  #define BUFFER_ID 2
  #define BUFFER_LENGTH 3	

  // Enumbs and defines
  #define CM730_ON  1
  #define DYN_ON CM730_ON  
  #define CM730_OFF 0
  #define DYN_OFF CM730_OFF
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
  #define MOTOR_VOLTAGE_SCALE 0.1
  #define MOTOR_POS_SCALE 1.0 / 4096.0 * 2.0 * M_PI
  #define MOTOR_SPEED_SCALE 0.11 / 60.0  * 2.0 * M_PI
  #define MOTOR_LOAD_SCALE 1.0
  #define MOTOR_TEMP_SCALE 1.0

  // Motor IDs
  #define ID_CM730 200
  #define ID_DARWIN ID_CM730
  #define ID_FT 100

  // Addresses 
  #define CM730_ADDRESS_DYN_POWER 24
  #define DYN_ADDRESS_DYN_POWER CM730_ADDRESS_DYN_POWER
  #define CM730_ADDRESS_ID 3
  
#define CM730_ADDRESS_VOLTAGE 50
  #define CM730_ADDRESS_STATUS_RETURN_LEVEL 16

  #define CM730_ADDRESS_IMU_START 38
  #define CM730_ADDRESS_IMU_LENGTH 12
  #define CM730_ADDRESS_READ_DATA_OFFSET 5

  #define CM730_ADDRESS_IMU_GYRO_Z 38
  #define CM730_ADDRESS_IMU_GYRO_Y 40
  #define CM730_ADDRESS_IMU_GYRO_X 42
  #define CM730_ADDRESS_IMU_ACC_X 44
  #define CM730_ADDRESS_IMU_ACC_Y 46
  #define CM730_ADDRESS_IMU_ACC_Z 48

  #define MX_ID 2  
  #define MX_ADDRESS_READ_DATA_OFFSET 5
  #define MX_ADDRESS_STATE_START 36
  #define MX_ADDRESS_STATE_LENGTH 8
  #define MX_ADDRESS_POS 36
  #define MX_ADDRESS_SPEED 38
  #define MX_ADDRESS_LOAD 40
  #define MX_ADDRESS_VOLTAGE 42
  #define MX_ADDRESS_TEMP 43
  #define MX_ADDRESS_DELAY 5
  #define MX_ADDRESS_STATUS_RETURN_LEVEL 16
  #define MX_ADDRESS_GOAL_POS 30

  #define FT_ADDRESS_STATUS_RETURN_LEVEL 16
  #define FT_ADDRESS_READ_DATA_OFFSET 5
  #define FT_ADDRESS_START 26
  #define FT_ADDRESS_LENGTH 10
  #define FT_ADDRESS_LEFT_X 26
  #define FT_ADDRESS_LEFT_Y 28
  #define FT_ADDRESS_RIGHT_X 30
  #define FT_ADDRESS_RIGHT_Y 32
  #define FT_ADDRESS_FSR_X 34
  #define FT_ADDRESS_FSR_Y 35
  #define FT_ADDRESS_VOLTAGE 42


  #define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"

  #define DARWIN_MOTOR_BROADCAST 0Xfe
  #define DARWIN_MOTOR_NUM 20
  #define DARWIN_MOTOR_MIN 1
  #define DARWIN_MOTOR_MAX 20

  #define ERROR 1
  #define NO_ERROR 0
  #define RAISED 1
  #define NOT_RAISED 0

  int sleep(double val);
  double time();
  int set_motor_delays();
  int set_motor_delays(int val);
  int set_motor_status_level();
  int set_motor_status_level(int val);
  int set_motor_pos(int id, double pos);
  uint16_t double2uint16(double val);
  uint8_t getMSB(uint16_t val);
  uint8_t getLSB(uint16_t val);
  double enc2rad(uint16_t val);
  double enc2radPerSec(uint16_t val);
  int set_ft_status_level();
  int set_ft_status_level(int val);
  int set_cm730_status_level();
  int set_cm730_status_level(int val);
  int set_status_level();
  int set_status_level(int val);

  int open();
  int open(const char* the_serial_port);
  int getch();
  int on();
  int on(int val);
  int off(int val);
  int off();
  int kbhit(void);
  int ping(int val);
  int get_imu_state();
  int get_imu_state_auto();
  int write(uint8_t id, uint8_t address, uint8_t d0);
  int update_imu(uint8_t val[]);
  int update_imu_setup();
  int update_imu_slow();
  int update_ft( uint8_t buff[] );
  int update_ft_setup();
  double int2double(uint16_t val); 
  double int2double(uint16_t val, int bit); 
  double uint2double(uint16_t val); 
  double ft_char2double(uint8_t val, int* err);
  uint8_t read1byte(uint8_t id, uint8_t address);
  int flush();
  uint16_t chars2uInt16(uint8_t d_lsb, uint8_t d_msb);
  int check_head( uint8_t buff[] );
  int check_checksum( uint8_t buff[] );
  int get_next_message( uint8_t buff[], int *the_length );
  int read(uint8_t id, uint8_t address);
  int read(uint8_t id, uint8_t address, uint8_t length);
  int read_buffer();
  int get_motor_state();
  int get_motor_state(int id);
  int get_motor_state_auto();
  int get_motor_state_auto(int n);
  int setup();
  int setup(const char* the_serial_port);
  int update_motor_state( uint8_t buff[]);
  void print_state_motor();
  void print_state_motor(int id);
  void print_state_motor_head();
  void print_state_imu();
  void print_state_imu_head();
  void print_state();
  double int2load(uint16_t val);

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

typedef struct motor_state_def {
	double pos;	
	double speed;
	double load;	
	double voltage;	
	double temp;		
}__attribute__((packed)) motor_state_def_t;

  motor_state_def_t motor_state[DARWIN_MOTOR_NUM+1];

void print_state()
{
  print_state_imu();
  print_state_motor();
}

void print_state_motor_head()
{
  printf("pos\t\t speed\t\t load\t\t voltage\t temp\n");
}
void print_state_motor(int id)
{
    motor_state_def_t ms = motor_state[id];
    printf("%f\t %f\t %f\t %f\t %f\n", ms.pos, ms.speed, ms.load, ms.voltage, ms.temp);
}
void print_state_motor()
{
  print_state_motor_head();
  for(int i = 0; i < (DARWIN_MOTOR_NUM+1); i++)
  {
    print_state_motor(i);
  }
}
void print_state_imu_head()
{
  printf("gyro_x\t\t gyro_y\t\t gyro_z\t\t acc_x\t\t acc_y\t\t acc_z\n");
}
void print_state_imu()
{
    print_state_imu_head();
    printf("%f\t %f\t %f\t %f\t %f\t %f\n", imu_gyro_x, imu_gyro_y, imu_gyro_z, imu_acc_x, imu_acc_y, imu_acc_z);
}
  int setup()
  {
    return setup(SERIAL_PORT_DEFAULT);
  }

  int setup(const char* the_serial_port)
  {
    memset(&motor_state, 0, sizeof(motor_state));
    return open(the_serial_port);
  }

  int set_motor_status_level()
  {
    return set_motor_status_level(2);
  }
  int set_motor_status_level(int val)
  {
    for( int i = 0; i < DARWIN_MOTOR_NUM; i++ )
    {
      write(i+1, MX_ADDRESS_STATUS_RETURN_LEVEL, val);
      sleep(1.0); 
    }
    return RETURN_OK;
  }
  
  int set_ft_status_level()
  {
    return set_ft_status_level(2);
  }

  int set_ft_status_level(int val)
  {
    return write(ID_FT, FT_ADDRESS_STATUS_RETURN_LEVEL, val);
  }

  int set_cm730_status_level()
  {
    return set_cm730_status_level(2);
  }

  int set_cm730_status_level(int val)
  {
    return write(ID_CM730, CM730_ADDRESS_STATUS_RETURN_LEVEL, val);
  }

  int set_status_level()
  {
    return set_status_level(2);
  }

  int set_status_level(int val)
  {
    int ret = 0;
    ret += set_motor_status_level(val);
    sleep(1.0);
    ret += set_ft_status_level(val);
    sleep(1.0);
    ret += set_cm730_status_level(val);
    if( ret >= 0 ) return RETURN_FAIL;
    else RETURN_OK;
  }

  int set_motor_delays(int val)
  {
    for( int i = 0; i < DARWIN_MOTOR_NUM; i++ )
    {
      int the_delay = i*val;
      if (the_delay > 250) the_delay = 250;
      write(i+1, MX_ADDRESS_DELAY, the_delay);
      sleep(0.1); 
    }
    return RETURN_OK;
  }

  int set_motor_delays()
  {
    for( int i = 0; i < DARWIN_MOTOR_NUM; i++ )
    {
      write(i+1, MX_ADDRESS_DELAY, i*10);
      sleep(0.1); 
    }
    return RETURN_OK;
  }


  int write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
  {
    return lofaro::do_write(id, address, d0, d1);
  }

  int write(uint8_t id, uint8_t address, uint8_t d0)
  {
    return lofaro::do_write(id, address, d0);
  }

  int on()
  {
    int ret =  write(ID_CM730, CM730_ADDRESS_DYN_POWER, CM730_ON);
    sleep(1.0);
    for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
    {
      ret += write(i, DYN_ADDRESS_DYN_POWER, DYN_ON);
    }

    if( ret > 0 ) return RETURN_FAIL;
    return RETURN_OK;
  }
  int off()
  {
    int ret = 0;
    for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
    {
      ret += write(i, DYN_ADDRESS_DYN_POWER, DYN_ON);
    }
    sleep(1.0);

    ret +=  write(ID_CM730, CM730_ADDRESS_DYN_POWER, CM730_OFF);

    if( ret > 0 ) return RETURN_FAIL;
    return RETURN_OK;
  }

  int on(uint8_t id)
  {
    int ret =  write(id, CM730_ADDRESS_DYN_POWER, CM730_ON);
    if ( ret == 0 ) return RETURN_OK;
    return RETURN_FAIL;
  }
  int off(uint8_t id)
  {
    int ret =  write(id, CM730_ADDRESS_DYN_POWER, CM730_OFF);
    if ( ret == 0 ) return RETURN_OK;
    return RETURN_FAIL;
  }

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

    if ( n == 0 ) return RETURN_FAIL;
/*
    printf("-----RX buffer = ");
    for( int i = 0; i < n; i++) printf("%x ",buff[i]);
    printf("\n");
*/

    bool do_run = true;
    while(do_run)
    {
      if( (RETURN_OK == check_head(buff)) & (RETURN_OK == check_checksum(buff)) )
      {
        uint8_t id = buff[BUFFER_ID];
        if      ( id == ID_CM730 )                                    update_imu(buff);
        else if ( id == ID_FT )                                       update_ft(buff);
        else if ( (id>=DARWIN_MOTOR_MIN) & (id<=DARWIN_MOTOR_MAX)  )  update_motor_state(buff);
      }
      if ( RETURN_FAIL == get_next_message(buff, &n) ) do_run = false;
    }

    return RETURN_OK;
  }

  int print_buff( uint8_t buff[], int n)
  {
      printf("Serial Buff Length = %d\n Buff=\n",n);
      for( int i = 0; i < n; i++ )
      {
        printf("%x ",(uint8_t)buff[i]);
      }
      printf("\n");
      printf("Is head ok %d\n",check_head(buff));
      printf("Is checksum ok %d\n",check_checksum(buff));

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

  double enc2rad(uint16_t val)
  {
    return (double)((double)((int32_t)val - 0x800)) / ( (double)(0x800) ) * M_PI;
  }

  double int2double(uint16_t val, int bit)
  {
    double tmp = 0.0;
    int bit_div = 2^(bit-1);
    if( val > bit_div ) tmp =(double)(val-bit_div)/(double)bit_div * -1.0;
    else                tmp =(double)(val        )/(double)bit_div *  1.0;
    return tmp;
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

  int get_imu_state()
  {
    return read( ID_CM730, CM730_ADDRESS_IMU_START, CM730_ADDRESS_IMU_LENGTH + 1 );
  }

  int get_imu_state_auto()
  {
      get_imu_state();
      bool do_loop = true;
      double tick = time();
      double tock = time();
      double dt = 0.0;
      while(do_loop)
      {
        int ret = read_buffer();
        if (ret == RETURN_OK) do_loop = false;
        tock = time();
        dt = tock - tick;
        if(dt > 0.001) do_loop = false;
        sleep(0.0001);
      }
      
    return RETURN_OK;
  }


  int get_motor_state_auto_i = DARWIN_MOTOR_MIN - 1;
  int get_motor_state_auto_n = 0;

  int get_motor_state_auto(int n)
  {
  //#define DARWIN_MOTOR_MIN 1
  //#define DARWIN_MOTOR_MAX 20
    // updates next motor only and switches every n cycles
    get_motor_state_auto_n++;
    if (get_motor_state_auto_n < n) return RETURN_OK;
    get_motor_state_auto_n = 0;

    get_motor_state_auto_i++;
    if(get_motor_state_auto_i > DARWIN_MOTOR_MAX) get_motor_state_auto_i = DARWIN_MOTOR_MIN - 1;

     get_motor_state(get_motor_state_auto_i);
     bool do_loop = true;
     double tick = time();
     double tock = time();
     double dt = 0.0;
     while(do_loop)
     {
        int ret = read_buffer();
        if (ret == RETURN_OK) do_loop = false;
        tock = time();
        dt = tock - tick;
        if(dt > 0.001) do_loop = false;
        sleep(0.0001);
     }
      
     return RETURN_OK;
  }

  int get_motor_state_auto()
  {
    for(int i = 0; i < DARWIN_MOTOR_NUM; i++)
    {
      get_motor_state(i+1);
      bool do_loop = true;
      double tick = time();
      double tock = time();
      double dt = 0.0;
      while(do_loop)
      {
        int ret = read_buffer();
        if (ret == RETURN_OK) do_loop = false;
        tock = time();
        dt = tock - tick;
        if(dt > 0.001) do_loop = false;
        sleep(0.0001);
      }
      
    }
    return RETURN_OK;
  }

  int get_motor_state()
  {
    int ret = 0;
    for( int i = 0; i < DARWIN_MOTOR_NUM; i++ )
    {
      ret += get_motor_state(i+1);
    }
    
    if (ret > 0) return RETURN_FAIL;
    return RETURN_OK;
  }

  int get_motor_state(int id)
  {
    int ret = lofaro::do_read(id, MX_ADDRESS_STATE_START, MX_ADDRESS_STATE_LENGTH);

    if( ret == 0 ) return RETURN_OK;
    return RETURN_FAIL;
  }

  int get_motor_state_bulk()
  {
    uint8_t buff_len = DARWIN_MOTOR_NUM * 3;
    uint8_t buff[buff_len];

    memset(&buff, 0, sizeof(buff));
    int mot_i = 1;    
    for( int i = 0; i < buff_len; i = i + 3)
    {
      buff[i]   = MX_ADDRESS_STATE_LENGTH;
      buff[i+1] = mot_i; 
      buff[i+2] = MX_ADDRESS_STATE_START;
      mot_i = mot_i + 1;
    }

    int ret = lofaro::do_read(DARWIN_MOTOR_BROADCAST, buff, buff_len);

    if( ret == 0 ) return RETURN_OK;
    return RETURN_FAIL;
  }

  int update_ft( uint8_t buff[])
  {
    if ( RETURN_OK != check_head(buff) ) return RETURN_FAIL;
    if ( RETURN_OK != check_checksum(buff) ) return RETURN_FAIL;

    // Assign the diata
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_LEFT_X) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_LEFT_X) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_left_x = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_LEFT_Y) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_LEFT_Y) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_left_y = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_RIGHT_X) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_RIGHT_X) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_right_x = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_RIGHT_Y) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_RIGHT_Y) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_right_y = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_FSR_X) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_FSR_X) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_fsr_x = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_FSR_Y) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_FSR_Y) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_fsr_y = chars2uInt16(b0, b1);
    
    // Assign the data

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

  uint16_t chars2uInt16(uint8_t d_lsb, uint8_t d_msb)
  {
    uint16_t d = 0;
    d = d | (uint16_t)d_lsb;
    d = d | ((uint16_t)d_msb << 8);
    return d;
  }

  int update_motor_state( uint8_t buff[])
  {
    if ( RETURN_OK != check_head(buff)     ) return RETURN_FAIL;
    if ( RETURN_OK != check_checksum(buff) ) return RETURN_FAIL;

/*
    uint8_t cs  = lofaro::get_checksum(buff);
    uint8_t cs2 = buff[ (MX_ADDRESS_TEMP - MX_ADDRESS_STATE_START) -4 ];
    printf("%x\t %x\n", cs, cs2);
    if ( cs != cs2 ) return RETURN_FAIL; 
*/
    uint8_t id = buff[MX_ID];
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    b0 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_POS) - MX_ADDRESS_STATE_START;
    b1 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_POS) - MX_ADDRESS_STATE_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_pos = chars2uInt16(b0, b1);

    b0 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_SPEED) - MX_ADDRESS_STATE_START;
    b1 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_SPEED) - MX_ADDRESS_STATE_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_speed = chars2uInt16(b0, b1);

    b0 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_LOAD) - MX_ADDRESS_STATE_START;
    b1 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_LOAD) - MX_ADDRESS_STATE_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_load = chars2uInt16(b0, b1);

    b0 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_VOLTAGE) - MX_ADDRESS_STATE_START;
    b0 = buff[b0];
    uint8_t buff_voltage = b0;

    b0 = (MX_ADDRESS_READ_DATA_OFFSET + MX_ADDRESS_TEMP) - MX_ADDRESS_STATE_START;
    b0 = buff[b0];
    uint8_t buff_temp = b0;

    motor_state[id].pos     = enc2rad(buff_pos);
    motor_state[id].speed   = enc2radPerSec(buff_speed);
    motor_state[id].load    = int2load(buff_load);
    motor_state[id].voltage = (double)buff_voltage       * MOTOR_VOLTAGE_SCALE;
    motor_state[id].temp    = (double)buff_temp          * MOTOR_TEMP_SCALE;

    return RETURN_OK;
  }

  double int2load(uint16_t val)
  {
    double dir = 1.0;
    int16_t mag_i = val & 0x3ff;
    int16_t dir_i = val & 0x400;

    if (dir_i) dir = -1.0;
    double the_out = (double)mag_i / (double)0x400 * dir * MOTOR_LOAD_SCALE;
    //double the_out = (double)mag_i / (double)0x400 * dir * MOTOR_SPEED_SCALE;

    return the_out;

    //return (double)( ((int32_t)(val) - 0x400) / 0x400 * MOTOR_SPEED_SCALE);
  }

  double enc2radPerSec(uint16_t val)
  {
    double dir = 1.0;
    int16_t mag_i = val & 0x3ff;
    int16_t dir_i = val & 0x400;

    if (dir_i) dir = -1.0;
    double the_out = (double)mag_i * dir * MOTOR_SPEED_SCALE;
    //double the_out = (double)mag_i / (double)0x400 * dir * MOTOR_SPEED_SCALE;

    return the_out;

    //return (double)( ((int32_t)(val) - 0x400) / 0x400 * MOTOR_SPEED_SCALE);
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
    return open(SERIAL_PORT_DEFAULT);
  }

  int open(const char* the_serial_port)
  {
    return lofaro::do_open(the_serial_port);
  }

  int off(int val)
  {
    return lofaro::do_write(val, CM730_ADDRESS_DYN_POWER,CM730_OFF); 
  }

  int on(int val)
  {
    return lofaro::do_write(val, CM730_ADDRESS_DYN_POWER,CM730_ON); 
  }

  int ping(int val)
  {
    return 0;
  }

  double time()
  {
    return lofaro::get_time();
  }

  uint8_t getMSB(uint16_t val)
  {
    return (uint8_t)((val >> 8) & 0xff);
  }

  uint8_t getLSB(uint16_t val)
  {
    return (uint8_t)(val & 0xff);
  }

  uint16_t double2uint16(double val)
  {
     uint16_t the_out = 0;

     the_out = (uint16_t)((val / M_PI * 0x800) + 0x800);

     if(the_out > 0xfff) the_out = 0xfff;
     else if( the_out < 0) the_out = 0;

     return the_out;
  }

  int set_motor_pos(int id, double pos)
  {
    uint16_t val = double2uint16(pos);
    uint8_t d0 = getLSB(val);
    uint8_t d1 = getMSB(val);
    return write(id, MX_ADDRESS_GOAL_POS, d0, d1);
  }
}

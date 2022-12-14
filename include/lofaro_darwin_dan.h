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


#include "lofaro_serial.h"
#include "lofaro_coms.h"



namespace darwin {

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
  int get_ft_state(uint8_t id);
  int get_ft_state_auto(uint8_t id);
  int get_ft_state_auto();
  int write(uint8_t id, uint8_t address);
  int write(uint8_t id, uint8_t address, uint8_t d0);
  int write_set(uint8_t id, uint8_t address);
  int write_set(uint8_t id, uint8_t address, uint8_t d0);
  int write_send();
  int write_send(uint8_t id);
  int update_imu(uint8_t val[]);
  int update_imu_setup();
  int update_imu_slow();
  int update_ft(uint8_t id, uint8_t buff[] );
  int update_ft_setup();
  double int2double(uint16_t val); 
  double int2double(uint16_t val, int bit); 
  double uint2double(uint16_t val); 
  double ft_char2double(uint8_t val, int* err);
  uint8_t read1byte(uint8_t id, uint8_t address);
  int flush();
  int flush_final();
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
  int setup(const char* the_serial_port, bool low_latency);
  int update_motor_state( uint8_t buff[]);
  void print_state_motor();
  void print_state_motor(int id);
  void print_state_motor_head();
  void print_state_imu();
  void print_state_imu_head();
  void print_state();
  void print_state_ft();
  void print_state_ft(uint8_t id);
  void print_state_ft_head();
  double int2load(uint16_t val);
  int set_ft_delay(int id, int val);
  int set_ft_delay(int val);


int flush_final()
{
  return lofaro::do_flush_final();
}

void print_state()
{
  print_state_imu();
  print_state_motor();
  print_state_ft();
}
void print_state_ft_head()
{
  printf("s0\t s1\t s2\t s3\t com_x\t com_y\t voltage\t raised_x\t raised_y\n");
}

void print_state_ft()
{
  print_state_ft(ID_FT_LEFT);
  print_state_ft(ID_FT_RIGHT);
}

void print_state_ft(uint8_t val)
{
  print_state_ft_head();
  int id = -1;
  if     (val == ID_FT_LEFT) { id = ENUM_FT_LEFT;  printf("FT LEFT:");  }
  else if(val == ID_FT_RIGHT){ id = ENUM_FT_RIGHT; printf("FT RIGHT:"); }
  else return;
  printf("%f, %f, %f, %f, %f, %f, %f, %d, %d\n",
          ft_state[id].s0,
          ft_state[id].s1,
          ft_state[id].s2,
          ft_state[id].s3,
          ft_state[id].x,
          ft_state[id].y,
          ft_state[id].voltage,
          ft_state[id].raised_x,
          ft_state[id].raised_y
        );
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
    return setup(the_serial_port, false);
  }

  int setup(const char* the_serial_port, bool low_latency)
  {
    memset(&darwin_data, 0, sizeof(darwin_data));
    memset(&motor_ref, 0, sizeof(motor_ref));
    memset(&motor_ref2, 0, sizeof(motor_ref2));
    memset(&motor_state, 0, sizeof(motor_state));
    memset(&ft_state, 0, sizeof(ft_state));
    memset(&imu_state, 0, sizeof(imu_state));
    int ret = open(the_serial_port);
    sleep(2.0);
    const char* head = "setserial ";
    const char* foot = " low_latency";
    char *s = new char[strlen(head) + strlen(foot) + strlen(the_serial_port) + 1];
    strcpy(s, head);
    strcat(s, the_serial_port);
    strcat(s, foot);
    if(low_latency)
    {
      std::system(s);
      //std::system("setserial /dev/ttyUSB0 low_latency");
      sleep(2.0);
    }
    return ret;
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
    int ret = write(ID_FT_LEFT, FT_ADDRESS_STATUS_RETURN_LEVEL, val);
    ret +=  write(ID_FT_RIGHT, FT_ADDRESS_STATUS_RETURN_LEVEL, val);
    if (ret > 0) return RETURN_FAIL;
    return RETURN_OK;
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
    return RETURN_OK;
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

  int set_ft_delay(int id, int val)
  {
    if( (id != ID_FT_RIGHT) | (id != ID_FT_LEFT) ) return RETURN_FAIL;
    if( (val < 0) | (val > 250) ) return RETURN_FAIL;

    write(id, FT_ADDRESS_DELAY, val);
    sleep(0.1); 

    return RETURN_OK;
  }
  int set_ft_delay(int val)
  {
    int ret = set_ft_delay(ID_FT_LEFT);
    ret    += set_ft_delay(ID_FT_RIGHT);
    if( ret > 0 ) return RETURN_FAIL;
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

  int write_send()
  {
    return lofaro::do_write_send(DYN_ID_ALL);
  }

  int write_send(uint8_t id)
  {
    return lofaro::do_write_send(id);
  }

  int write_set(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
  {
    return lofaro::do_write_set(id, address, d0, d1);
  }

  int write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
  {
    return lofaro::do_write(id, address, d0, d1);
  }

  int write_set(uint8_t id, uint8_t address, uint8_t d0)
  {
    return lofaro::do_write_set(id, address, d0);
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
      sleep(0.05);
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
    lofaro::do_read_buffer(buff, &n);

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
        else if ( id == ID_FT_LEFT )                                  update_ft(id, buff);
        else if ( id == ID_FT_RIGHT )                                 update_ft(id, buff);
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

      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wrestrict"
      memcpy(buff, buff + 1*sizeof(buff[0]), (1024-1)*sizeof(buff[0]) );
      #pragma GCC diagnostic pop

      //memcpy(buff, buff + 1*sizeof(buff[0]), (1024-1)*sizeof(buff[0]) );
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

  int get_ft_state(uint8_t id)
  {
    return read( id, FT_ADDRESS_START, FT_ADDRESS_LENGTH + 1 );
  }

  int get_ft_state_auto(uint8_t id)
  {
    if( (id != ID_FT_LEFT) & (id != ID_FT_RIGHT) ) return RETURN_FAIL;
    get_ft_state(id);
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
      if(dt > 0.0015) do_loop = false;
      sleep(0.0001);
    }
      
    return RETURN_OK;
  }

  int get_ft_state_auto()
  {
    int ret = get_ft_state_auto(ID_FT_RIGHT);
    ret    += get_ft_state_auto(ID_FT_LEFT);
    if (ret > 0) return RETURN_FAIL;
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

  int update_ft(uint8_t id, uint8_t buff[])
  {
    if ( RETURN_OK != check_head(buff) ) return RETURN_FAIL;
    if ( RETURN_OK != check_checksum(buff) ) return RETURN_FAIL;

    // Assign the diata
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S1) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S1) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_s1 = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S2) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S2) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_s2 = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S3) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S3) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_s3 = chars2uInt16(b0, b1);
    
    b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S4) - FT_ADDRESS_START;
    b1 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_S4) - FT_ADDRESS_START + 1;
    b0 = buff[b0];
    b1 = buff[b1];
    uint16_t buff_s4 = chars2uInt16(b0, b1);
    
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
  //#define FT_ADDRESS_VOLTAGE 42
    
   b0 = (FT_ADDRESS_READ_DATA_OFFSET + FT_ADDRESS_VOLTAGE) - FT_ADDRESS_START;
   uint8_t vol = b0;
    // Assign the data

    double ft_0   = int2double(buff_s1) * FT_SCALE;
    double ft_1   = int2double(buff_s2) * FT_SCALE;
    double ft_2   = int2double(buff_s3) * FT_SCALE;
    double ft_3   = int2double(buff_s4) * FT_SCALE;

    double ft_x    = ft_char2double(buff_fsr_x, &ft_fsr_raised_x) * FSR_SCALE_X;
    double ft_y    = ft_char2double(buff_fsr_y, &ft_fsr_raised_y) * FSR_SCALE_Y;
    double ft_v    = (double)vol /10.0;

    int the_index = -1;

    if     ( id == ID_FT_LEFT  ) the_index = ENUM_FT_LEFT;
    else if( id == ID_FT_RIGHT ) the_index = ENUM_FT_RIGHT;
    else return RETURN_FAIL;

    ft_state[the_index].s0 = ft_0;
    ft_state[the_index].s1 = ft_1;
    ft_state[the_index].s2 = ft_2;
    ft_state[the_index].s3 = ft_3;
    ft_state[the_index].x  = ft_x;
    ft_state[the_index].y  = ft_y;
    ft_state[the_index].raised_x = ft_fsr_raised_x;
    ft_state[the_index].raised_y = ft_fsr_raised_y;
    ft_state[the_index].voltage  = ft_v;

    darwin_data.ft_state[the_index].s0 = ft_0;
    darwin_data.ft_state[the_index].s1 = ft_1;
    darwin_data.ft_state[the_index].s2 = ft_2;
    darwin_data.ft_state[the_index].s3 = ft_3;
    darwin_data.ft_state[the_index].x  = ft_x;
    darwin_data.ft_state[the_index].y  = ft_y;
    darwin_data.ft_state[the_index].raised_x = ft_fsr_raised_x;
    darwin_data.ft_state[the_index].raised_y = ft_fsr_raised_y;
    darwin_data.ft_state[the_index].voltage  = ft_v;

    //darwin_data.ft_state = ft_state;

    return RETURN_OK;
  }

  double ft_char2double(uint8_t val, int* err)
  {
    if( val == 255)
    {
      *err = RAISED;
      return 0.0;
    }
    
    double the_out = ((double)val - 127.0) / 127.0;
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

    darwin_data.motor_state[id].pos     = enc2rad(buff_pos);
    darwin_data.motor_state[id].speed   = enc2radPerSec(buff_speed);
    darwin_data.motor_state[id].load    = int2load(buff_load);
    darwin_data.motor_state[id].voltage = (double)buff_voltage       * MOTOR_VOLTAGE_SCALE;
    darwin_data.motor_state[id].temp    = (double)buff_temp          * MOTOR_TEMP_SCALE;
    //darwin_data.motor_state = motor_state;

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
    imu_state.gyro_x = imu_gyro_x;
    imu_state.gyro_y = imu_gyro_y;
    imu_state.gyro_z = imu_gyro_z;
    imu_state.acc_x  = imu_acc_x;
    imu_state.acc_y  = imu_acc_y;
    imu_state.acc_z  = imu_acc_z;
    imu_state.voltage = voltage;

    darwin_data.imu_state.gyro_x = imu_gyro_x;
    darwin_data.imu_state.gyro_y = imu_gyro_y;
    darwin_data.imu_state.gyro_z = imu_gyro_z;
    darwin_data.imu_state.acc_x  = imu_acc_x;
    darwin_data.imu_state.acc_y  = imu_acc_y;
    darwin_data.imu_state.acc_z  = imu_acc_z;
    darwin_data.imu_state.voltage = voltage;

    //darwin_data.imu_state = imu_state;
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
    sleep(0.5);
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

  int set_motor_pos_set(int id, double pos)
  {
    uint16_t val = double2uint16(pos);
    uint8_t d0 = getLSB(val);
    uint8_t d1 = getMSB(val);
    return write_set(id, MX_ADDRESS_GOAL_POS, d0, d1);
  }

  int set_motor_pos(int id, double pos)
  {
    uint16_t val = double2uint16(pos);
    uint8_t d0 = getLSB(val);
    uint8_t d1 = getMSB(val);
    return write(id, MX_ADDRESS_GOAL_POS, d0, d1);
  }
}

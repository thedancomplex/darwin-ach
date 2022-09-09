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


namespace lofaro {
#include "lofaro_utils.h"

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
void do_flush();

struct termios tty;

int check_serial()
{
  if(serial_port <= 0) return 1;
  return 0;
}

int do_open()
{
  return do_open(SERIAL_PORT_DEFAULT);
}

int do_open(const char* the_serial_port)
{
//  struct termios {
//	tcflag_t c_iflag;		/* input mode flags */
//	tcflag_t c_oflag;		/* output mode flags */
//	tcflag_t c_cflag;		/* control mode flags */
//	tcflag_t c_lflag;		/* local mode flags */
//	cc_t c_line;			/* line discipline */
//	cc_t c_cc[NCCS];		/* control characters */
// };

  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct
//  struct termios tty;

bool do_robotis = true;

if(do_robotis)
{
  serial_port = open(the_serial_port, O_RDWR|O_NOCTTY|O_NONBLOCK);
  bzero(&tty, sizeof(tty)); // clear struct for new port settings

  tty.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
  tty.c_iflag = IGNPAR;
  tty.c_oflag      = 0;
  tty.c_lflag      = 0;
  tty.c_cc[VTIME]  = 0;
  tty.c_cc[VMIN]   = 0;

  // clean the buffer and activate the settings for the port
  tcflush(serial_port, TCIFLUSH);
  tcsetattr(serial_port, TCSANOW, &tty);

  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return 1;
  }
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }
  return 0;
}
else
{
  serial_port = open(the_serial_port, O_RDWR);
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  // tty.c_cflag |= PARENB;  // Set parity bit, enabling parity

  // tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
  // tty.c_cflag |= CS5; // 5 bits per byte
  // tty.c_cflag |= CS6; // 6 bits per byte
  // tty.c_cflag |= CS7; // 7 bits per byte
  tty.c_cflag |= CS8; // 8 bits per byte (most common)

  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  // tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;  // Disable Canonical mode (i.e. do not wait for a new line

  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl




  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  // tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;


  // cfsetispeed(&tty, B1000000);
  // cfsetspeed(&tty, B1000000);

  cfsetispeed(&tty, B1000000);
  cfsetospeed(&tty, B1000000);
  // Check for errors
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return 1;
  }

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined

//  tcflush(serial_port, TCIFLUSH);

  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return 0;
}

 // serial_port = open("/dev/ttyUSB0", O_RDWR);
  return 1;
}

int do_close()
{
  close(serial_port);
  return 0;
}

int do_write_set(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
{
  // Get ID Example
  uint8_t length = 2 + 2 + 1;
  uint8_t instruction = DYN_WRITE_REG;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, d0, d1};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/

  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}

int do_write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1)
{
  // Get ID Example
  uint8_t length = 2 + 2 + 1;
  uint8_t instruction = DYN_WRITE;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, d0, d1};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}

int do_write_set(uint8_t id, uint8_t address, uint8_t d0)
{
  // Get ID Example
  uint8_t length = 2 + 2;
  uint8_t instruction = DYN_WRITE_REG;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, d0};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}

int do_write_send()
{
  return do_write_send( DYN_ID_ALL );
}

int do_write_send(uint8_t id)
{
  // Get ID Example
  uint8_t length = 2;
  uint8_t instruction = DYN_ACTION;
  uint8_t msg[] = { 255, 255, id, length, instruction};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}


int do_write(uint8_t id, uint8_t address, uint8_t d0)
{
  // Get ID Example
  uint8_t length = 2 + 2;
  uint8_t instruction = DYN_WRITE;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, d0};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}


void do_flush()
{
  return;
}

int do_read(uint8_t id, uint8_t buff[], uint8_t buff_length)
{

  uint8_t length = 2 + buff_length + 1 ;
  uint8_t msg_length_full = length + 3 + 1 ;
  uint8_t msg[msg_length_full + 10];
  memset(&msg, 0, sizeof(msg));
  msg[0] = 0xff;
  msg[1] = 0xff;
  msg[2] = id;
  msg[3] = length;
  msg[4] = DYN_READ_BULK;
  msg[5] = 0x00;
  int ii = 0;
  for( int i = 0; i < buff_length; i++ )
  {
    ii = i + 6;
    msg[ii] = buff[i];
  }

  uint8_t the_checksum = get_checksum(msg);
  msg[msg_length_full - 1] = the_checksum;

/*
  printf("checksum = %x\n", the_checksum);

  printf("Write Buffer = ");
  for( int i = 0; i < buff_length + 10; i++ ) printf("%x ",(uint8_t)msg[i]);
  printf("\n");
*/
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(msg));
  do_flush();

  return 0;
}

int do_read(uint8_t id, uint8_t address, uint8_t length_read)
{
  // Get ID Example
  uint8_t length = 2 + 2;
  uint8_t instruction = DYN_READ;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, length_read};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}

int do_read(uint8_t id, uint8_t address)
{
  // Get ID Example
  uint8_t length = 2 + 2;
  uint8_t instruction = DYN_READ;
  uint8_t msg[] = { 255, 255, id, length, instruction, address, 1};
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++) buff[i] = msg[i];

  buff[buff_length-1] = the_checksum;
/*
  printf("Write Buffer = ");
  for( int i = 0; i < n; i++ ) printf("%x ",(uint8_t)buff[i]);
  printf("\n");
*/
  
  if(check_serial()) return 1;
  write(serial_port, buff, sizeof(buff));
  do_flush();
//  do_read();
  return 0;
}

int do_read_buffer(uint8_t buff[1024], int *the_length)
{
  uint8_t read_buf[1024];

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  if(check_serial()) return 1;
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  *the_length = n;

  buff = read_buf;

  if( n > 0) return 0;
  return 1;

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
  printf("Serial Buff Length = %d\n Buff=\n",n);
  for( int i = 0; i < n; i++ )
  {
    printf("%x ",(uint8_t)read_buf[i]);
  }
  printf("\n");

  if( n > 2 )
  {
      if( (read_buf[0] == 0xff) & (read_buf[1] == 0xff) )
      {
         if( get_checksum(read_buf) == read_buf[ read_buf[3] + 2 + 1] ) printf("Checksum OK\n");
         else printf("Checksum FAIL\n");
      }
  }
  printf("\n");


  return 0;
}

void print_buffer(uint8_t *buff)
{
  int n = 10;
  //printf("Serial Buff Length = %d\n Buff=\n",n);

  std::cout << buff << std::endl;
  
  for( int i = 0; i < n; i++ )
  {
//    printf("%x ",(uint8_t)buff[i]);
  }
  printf("\n");
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



}

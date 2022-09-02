// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdint>

namespace lofaro {

#include "lofaro_utils.h"

#define SERIAL_PKT_LENGTH 3


#define DYN_PING          0x01
#define DYN_READ          0x02
#define DYN_WRITE         0x03
#define DYN_WRITE_REG     0x04
#define DYN_ACTION        0x05
#define DYN_FACTORY_RESET 0x06
#define DYN_REBOOT        0x08 
#define DYN_WRITE_SYNC    0x83
#define DYN_READ_BULK     0x92

int serial_port = 0;

uint8_t get_checksum(uint8_t *rxpacket);
int     do_open();
int     do_write(uint8_t id, uint8_t address, uint8_t d0);
int     do_write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1);
int     do_write(uint8_t id, uint8_t address, uint8_t length, uint8_t[] dn);
int     do_read();


int do_open()
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
  struct termios tty;

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

  serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Check for errors
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return 1;
  }

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return 0;
}

int     do_write(uint8_t id, uint8_t address, uint8_t d0, uint8_t d1);
int     do_write(uint8_t id, uint8_t address, uint8_t length, uint8_t[] dn);

int do_write(uint8_t id, uint8_t address, uint8_t d0)
{
  // Get ID Example
  uint8_t length = 1 + 2;
  uint8_t instruction_read = 2;
  uint8_t the_address = address;
  uint8_t msg[] = { 255, 255, 200, length, instruction_read, 3 };
  uint8_t the_checksum = get_checksum(msg);
  uint8_t buff_length = sizeof(msg)/sizeof(msg[0]) + 1;
  uint8_t buff[buff_length];
  for (int i = 0; i < (buff_length-1); i++)
  {
    buff[i] = msg[i];
  }
  buff[buff_length-1] = the_checksum;
  
  write(serial_port, buff, sizeof(buff));
//  do_read();
  return 0;
}

int do_read()
{
  char read_buf [256];

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
  printf("Serial Buff Length = %d\n Buff=\n",n);
  for( int i = 0; i < n; i++ )
  {
    printf("%x ",(uint8_t)read_buf[i]);
  }
  printf("\n");
  return 0;
}

void print_buffer(uint8_t *buff)
{
  int n = 10;
  printf("Serial Buff Length = %d\n Buff=\n",n);
  for( int i = 0; i < n; i++ )
  {
    printf("%x ",(uint8_t)buff[i]);
  }
  printf("\n");
}

uint8_t get_checksum(uint8_t *rxpacket)
{
  uint8_t checksum    = 0;
  uint8_t wait_length = rxpacket[SERIAL_PKT_LENGTH] + SERIAL_PKT_LENGTH + 1;


  // calculate checksum
  for (uint16_t i = 2; i < wait_length - 1; i++)   // except header, checksum
    checksum += rxpacket[i];
    checksum = ~checksum;
  return checksum;
}



}

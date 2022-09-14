#ifndef DARWIN_TYPES
#include "lofaro_types.h"
#endif

#ifndef LOFARO_UTILS
#include "lofaro_utils.h"
#endif

#include "dynamixel_sdk/dynamixel_sdk.h"

class DarwinLofaro
{
  public:
    DarwinLofaro();

    /* Setup system with default values */
    int setup();

    /* Setup system with optional port */
    int setup(char *port);

    /* Setup system with low latency flag */
    int setup(bool low_latency);

    /* Setup system with optional port and low latency flag */
    int setup(char *port, bool low_latency);

  /* Set Motor Position */
  int setMotor(int mot, double val);

  /* Get Motor State */
  int getMotor(int id);

  /* Stage Motor Position */
  int stageMotor(int mot, double val);

  /* Send staged motor positions to all motors */
  int putMotor();

  /* Send staged motor positions to motor "mot" */
  int putMotor(int mot);

  /* Get IMU State */
  int getImu();

  /* Get Left and Right FT states */
  int getFt();

  /* Get "id" FT state */
  int getFt(int id);

  /* Turn on all */
  int on();

  /* Turn on "id" */
  int on(int id);

  /* Turn off all */
  int off();

  /* Turn off "id" */
  int off(int id);





  /* State and Reference Data */
  static darwin_data_def_t darwin_data;

  private:
    /* Open Port */
    int open();

    /* Open Port and change port number */
    int open(char *port);

    /* Sets low-latency for serial port */
    int setLowLatency(char* the_serial_port, bool low_latency);

    LofaroUtils* lut = new LofaroUtils();

    /* IMU specific int to double */
    double int2double(uint16_t val);





};


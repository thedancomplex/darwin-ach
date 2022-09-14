#ifdef DYN_SERIAL
#include "lofaro_darwin_dyn.h"
#endif

#ifndef DYN_SERIAL
#include "lofaro_darwin_dan.h"
#endif




class DarwinLofaro
{
  public:

  /* Open Port */
  int open();

  /* Open Port and change port number */
  int open(char *port);

  /* Setup system with default values */
  int setup();

  /* Setup system with optional port */
  int open(char *port);

  /* Setup system with low latency flag */
  int open(bool low_latency);

  /* Setup system with optional port and low latency flag */
  int open(char *port, bool low_latency);

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

  private:


};


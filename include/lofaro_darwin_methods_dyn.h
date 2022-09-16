#define DARWIN_METHODS_DYN 1


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

/* init */
DarwinLofaro::DarwinLofaro()
{
  return;
}

/* Open Port */
int DarwinLofaro::open()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->open(cstr);
  delete [] cstr;
  return ret;
}

/* Open Port and change port number */
int DarwinLofaro::open(const char *port)
{
  portHandler->setPacketTimeout(0.001);

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return RETURN_FAIL;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return RETURN_FAIL;
  }

  return RETURN_OK;
}

/* Setup system with default values */
int DarwinLofaro::setup()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, SERIAL_PORT_LOW_LATENCY_DEFAULT);
  delete [] cstr;
  return ret;
}

int DarwinLofaro::setup(std::string str)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port */
int DarwinLofaro::setup(const char *port)
{
  return this->setup(port, SERIAL_PORT_LOW_LATENCY_DEFAULT);
}

/* Setup system with low latency flag */
int DarwinLofaro::setup(bool low_latency)
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

int DarwinLofaro::setup(std::string str, bool low_latency)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port and low latency flag */
int DarwinLofaro::setup(const char *port, bool low_latency)
{
  memset(&darwin_data, 0, sizeof(darwin_data));
  this->open(port);
  this->setLowLatency(port, low_latency);
  return RETURN_OK;
}

/* Sets low-latency for serial port */
int DarwinLofaro::setLowLatency(const char* the_serial_port, bool low_latency)
{
  if( low_latency )
  {
    const char* head = "setserial ";
    const char* foot = " low_latency";
    char *s = new char[strlen(head) + strlen(foot) + strlen(the_serial_port) + 1];
    strcpy(s, head);
    strcat(s, the_serial_port);
    strcat(s, foot);
    std::system(s);
    sleep(2.0);
    return RETURN_OK;
  }
  return RETURN_FAIL;
}

/* Get IMU State */
int DarwinLofaro::getImu()
{
  bool dxl_addparam_result = false;               // addParam result
  groupBulkReadImu.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = groupBulkReadImu.addParam(ID_CM730, 
                                                        CM730_ADDRESS_IMU_START, 
                                                        CM730_ADDRESS_IMU_LENGTH+1);

  if (dxl_addparam_result != true) return RETURN_FAIL;
  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = groupBulkReadImu.txRxPacket();
  packetHandler->getTxRxResult(dxl_comm_result);
  if (groupBulkReadImu.getError(ID_CM730, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = groupBulkReadImu.isAvailable(ID_CM730, 
                                                          CM730_ADDRESS_IMU_START, 
                                                          CM730_ADDRESS_IMU_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;

  // Assign the data
  uint16_t buff_gyro_x  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_X, 2);
  uint16_t buff_gyro_y  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Y, 2);
  uint16_t buff_gyro_z  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Z, 2);
  uint16_t buff_acc_x   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_X, 2);
  uint16_t buff_acc_y   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Y, 2);
  uint16_t buff_acc_z   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Z, 2);
  uint8_t  buff_voltage = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_VOLTAGE, 1);

  this->darwin_data.imu.gyro_x  = this->int2double(buff_gyro_x) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_y  = this->int2double(buff_gyro_y) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_z  = this->int2double(buff_gyro_z) * IMU_GYRO_SCALE;
  this->darwin_data.imu.acc_x   = this->int2double(buff_acc_x)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_y   = this->int2double(buff_acc_y)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_z   = this->int2double(buff_acc_z)  * IMU_ACC_SCALE;
  this->darwin_data.imu.voltage = (double)buff_voltage    / VOLTAGE_SCALE;
 
  return RETURN_OK;
}

/* Turn on all */
int DarwinLofaro::on()
{
  int ret = 0;
  ret += this->on(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->on(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->on(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->on(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}



int DarwinLofaro::on(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_ON, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned on \n");
    }
    return RETURN_OK;;
}

int DarwinLofaro::sleep(double val)
{
  return this->lut->sleep(val);
}

/* Turn off all */
int DarwinLofaro::off()
{
  int ret = 0;
  ret += this->off(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->off(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->off(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->off(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}


/* Turn off "id" */
int DarwinLofaro::off(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_OFF, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned off \n");
    }
    return RETURN_OK;
}


double DarwinLofaro::int2double(uint16_t val)
{
  double the_out = (double)((int32_t)val - 512) / 1023.0;
  return the_out;
}



/* Stops and turns off everything */
int DarwinLofaro::stop()
{
  int ret = off();
  ret += close();
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;  
}

/* Closes port */
int DarwinLofaro::close()
{
  // Close port
  portHandler->closePort();
  return RETURN_OK;
}


int DarwinLofaro::sleep()
{
  return this->lut->sleep();
}

int DarwinLofaro::rate(double hz)
{
  return this->lut->rate(hz);
}

double DarwinLofaro::time()
{
  return this->lut->getTime();
}



/* Get Left and Right FT states */
int DarwinLofaro::getFt()
{ 
  int ret = getFt(ID_FT_LEFT);
  ret    += getFt(ID_FT_RIGHT);
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK; 
}


/* FT specific char 2 double */
double DarwinLofaro::ft_char2double(uint8_t val, int* err)
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



/* Get "id" FT state */
int DarwinLofaro::getFt(int id)
{ 

  int the_index = -1;
  if      (id == ID_FT_LEFT)  the_index = ENUM_FT_LEFT;
  else if (id == ID_FT_RIGHT) the_index = ENUM_FT_RIGHT;
  else return RETURN_FAIL;


  bool dxl_addparam_result = false;               // addParam result
  groupBulkReadFt.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = groupBulkReadFt.addParam(id, FT_ADDRESS_START, FT_ADDRESS_LENGTH);
  if (dxl_addparam_result != true) return RETURN_FAIL;


  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = groupBulkReadFt.txRxPacket();
  packetHandler->getTxRxResult(dxl_comm_result);
  if (groupBulkReadFt.getError(id, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = groupBulkReadFt.isAvailable(id, FT_ADDRESS_START, FT_ADDRESS_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;

  // Assign the data
  uint16_t buff_s1       = groupBulkReadFt.getData(id, FT_ADDRESS_S1, 2);
  uint16_t buff_s2       = groupBulkReadFt.getData(id, FT_ADDRESS_S2, 2);
  uint16_t buff_s3       = groupBulkReadFt.getData(id, FT_ADDRESS_S3, 2);
  uint16_t buff_s4       = groupBulkReadFt.getData(id, FT_ADDRESS_S4, 2);
  uint16_t buff_fsr_x    = groupBulkReadFt.getData(id, FT_ADDRESS_FSR_X, 2);
  uint16_t buff_fsr_y    = groupBulkReadFt.getData(id, FT_ADDRESS_FSR_Y, 2);
  uint8_t  buff_voltage  = groupBulkReadFt.getData(id, FT_ADDRESS_VOLTAGE, 1);


  this->darwin_data.ft[the_index].s0   = this->int2double(buff_s1)  * FT_SCALE;
  this->darwin_data.ft[the_index].s1   = this->int2double(buff_s2)  * FT_SCALE;
  this->darwin_data.ft[the_index].s2   = this->int2double(buff_s3)  * FT_SCALE;
  this->darwin_data.ft[the_index].s3   = this->int2double(buff_s4)  * FT_SCALE;

  int ft_fsr_raised_x = 0;
  int ft_fsr_raised_y = 0;

  this->darwin_data.ft[the_index].x    = this->ft_char2double(buff_fsr_x, &ft_fsr_raised_x) * FSR_SCALE_X;
  this->darwin_data.ft[the_index].y    = this->ft_char2double(buff_fsr_y, &ft_fsr_raised_y) * FSR_SCALE_Y;

  this->darwin_data.ft[the_index].raised_x = ft_fsr_raised_x;
  this->darwin_data.ft[the_index].raised_y = ft_fsr_raised_y;
   
  this->darwin_data.ft[the_index].voltage  = (double)buff_voltage / VOLTAGE_SCALE;

return RETURN_OK; 
}






  /* Set Motor Position */
  int setMotor(int mot, double val)
  { return RETURN_OK; }

  /* Get Motor State */
  int getMotor(int id)
  { return RETURN_OK; }

  /* Stage Motor Position */
  int stageMotor(int mot, double val)
  { return RETURN_OK; }

  /* Send staged motor positions to all motors */
  int putMotor()
  { return RETURN_OK; }

  /* Send staged motor positions to motor "mot" */
  int putMotor(int mot)
  { return RETURN_OK; }



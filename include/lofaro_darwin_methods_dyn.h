#define DARWIN_METHODS_DYN 1

/* Open Port */
int DarwinLofaro::open()
{
  return this->open(SERIAL_PORT_DEFAULT);
}

/* Open Port and change port number */
int DarwinLofaro::open(char *port)
{
  this->portHandler->setPacketTimeout(0.001);

  // Open port
  if (this->portHandler->openPort())
  {
    PORT_STATUS = true;
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return RETURN_FAIL;
  }

  // Set port baudrate
  if (this->portHandler->setBaudRate(BAUDRATE))
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
  return this->setup(SERIAL_PORT_DEFAULT, SERIAL_PORT_LOW_LATENCY_DEFAULT);
}

/* Setup system with optional port */
int DarwinLofaro::setup(char *port)
{
  return this->setup(port, SERIAL_PORT_LOW_LATENCY_DEFAULT);
}

/* Setup system with low latency flag */
int DarwinLofaro::setup(bool low_latency)
{
  return this->setup(SERIAL_PORT_DEFAULT, low_latency);
}

/* Setup system with optional port and low latency flag */
int DarwinLofaro::setup(char *port, bool low_latency)
{
  memset(&darwin_data, 0, sizeof(darwin_data));
  this->open(port);
  this->setLowLatency(low_latency);
  return RETURN_OK;
}

/* Sets low-latency for serial port */
int DarwinLofaro::setLowLatency(bool low_latency)
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
int getImu()
{
  this->groupBulkReadImu.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = this->groupBulkReadImu.addParam(ID_CM730, 
                                                        CM730_ADDRESS_IMU_START, 
                                                        CM730_ADDRESS_IMU_LENGTH+1);

  if (dxl_addparam_result != true) return RETURN_FAIL;
  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = this->groupBulkReadImu.txRxPacket();
  this->packetHandler->getTxRxResult(dxl_comm_result);
  if (this->groupBulkReadImu.getError(ID_CM730, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = this->groupBulkReadImu.isAvailable(ID_CM730, 
                                                          CM730_ADDRESS_IMU_START, 
                                                          CM730_ADDRESS_IMU_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;

  // Assign the data
  uint16_t buff_gyro_x  = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_X, 2);
  uint16_t buff_gyro_y  = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Y, 2);
  uint16_t buff_gyro_z  = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Z, 2);
  uint16_t buff_acc_x   = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_X, 2);
  uint16_t buff_acc_y   = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Y, 2);
  uint16_t buff_acc_z   = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Z, 2);
  uint8_t  buff_voltage = this->groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_VOLTAGE, 1);

  this->darwin_data.imu.gyro_x  = int2double(buff_gyro_x) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_y  = int2double(buff_gyro_y) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_z  = int2double(buff_gyro_z) * IMU_GYRO_SCALE;
  this->darwin_data.imu.acc_x   = int2double(buff_acc_x)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_y   = int2double(buff_acc_y)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_z   = int2double(buff_acc_z)  * IMU_ACC_SCALE;
  this->darwin_data.imu.voltage = (double)buff_voltage    / VOLTAGE_SCALE;
 
  return RETURN_OK;
}

/* Turn on all */
int DarwinLofaro::on()
{
  int ret = 0;
  ret += this->on(ID_CM730);
  lut.sleep(2.0);
  ret += this->on(ID_FT_RIGHT);
  lut.sleep(0.1);
  ret += this->on(ID_FT_LEFT);
  lut.sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->on(i);
    lut.sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}



int DarwinLofaro::on(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = this->packetHandler->write1ByteTxRx(this->portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_ON, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", this->packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", this->packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned on \n");
    }
    return RETURN_OK;;
}

/* Turn off all */
int off()
{
  int ret = 0;
  ret += this->off(ID_CM730);
  lut.sleep(2.0);
  ret += this->off(ID_FT_RIGHT);
  lut.sleep(0.1);
  ret += this->off(ID_FT_LEFT);
  lut.sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->off(i);
    lut.sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}


/* Turn off "id" */
int off(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = this->packetHandler->write1ByteTxRx(this->portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_OFF, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", this->packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", this->packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned off \n");
    }
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

  /* Get Left and Right FT states */
  int getFt()
  { return RETURN_OK; }

  /* Get "id" FT state */
  int getFt(int id)
  { return RETURN_OK; }


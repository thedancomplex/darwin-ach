int DarwinLofaroLegacyRos2::update_50hz()
{
  return this->update_50hz(HZ_NULL);
}

int DarwinLofaroLegacyRos2::update_50hz(int mode)
{
  /* Designed to update at 50hz */
 
  int ret = 0;

  if( mode == HZ_IMU )
  {
    /* Set Ref */
    ret += this->dl->stageMotor();
    ret += this->dl->putMotor();

    /* Get State */
    ret += this->dl->getImu();    
  }
  else
  {
    /* Set Ref */
    ret += this->dl->stageMotor();
    ret += this->dl->putMotor();

    /* Get State */
    ret += this->dl->getImu();
    ret += this->dl->getFt();

    ret += this->dl->getMotorSlow(1);
  }
  if ( ret > 0 ) ret = 1;
  return ret;
}


int DarwinLofaroLegacyRos2::update_125hz()
{
  /* Designed to update at 125hz (8ms) update rate */
  /* Lower body, IMU, and FT get priority */
  
  int ret = 0;

  /* Stage all Motors */
  ret += this->dl->stageMotor();

  /* Send to all motors */
  ret += this->dl->putMotor();

  /* Get State */
  ret += this->dl->getImu();

  if ( ret > 0 ) ret = 1;
  return ret;
}


int upper_i = 0;
int ft_i = 0;
int DarwinLofaroLegacyRos2::update_100hz()
{
  return this->update_100hz(HZ_MODE_MOTORS);
}

int DarwinLofaroLegacyRos2::update_100hz(int mode)
{
  /* Designed to update at 100hz (10ms) update rate */
  /* Lower body, IMU, and FT get priority */

  int ret = 0;

  if( mode == HZ_MODE_MOTORS_AND_STATE )
  {
    const int LOWER_START = 7;
    const int LOWER_END   = 18;
  
    int upper_array[] = {1,2,3,4,5,6,19,20};
    int UPPER_LENGTH  = 8;
 
    /* Set one upper Ref per cycle */
    upper_i++;
    if(upper_i >= UPPER_LENGTH) upper_i = 0;
    ret += this->dl->stageMotor(upper_array[upper_i]);

    /* Always stage lower body */
    for(int i = LOWER_START; i <= LOWER_END; i++)
    {
      ret += this->dl->stageMotor(i);
    }
  }
  else if( mode == HZ_MODE_MOTORS )
  {
    /* Stage all Motors */
    ret += this->dl->stageMotor();
  }

  /* Send to all motors */
  ret += this->dl->putMotor();

  /* Get IMU State */
  ret += this->dl->getImu();

  /* Get Ft State every other cycle */
  if(ft_i == 0) ft_i = 1;
  else ft_i = 0;
  ret += this->dl->getFt(ft_i);

  if( mode == HZ_MODE_MOTORS_AND_STATE )
  {
    /* Get motor state (one per cycle) */
    ret += this->dl->getMotorSlow(1);
  }

  if ( ret > 0 ) ret = 1;
  return ret;
}


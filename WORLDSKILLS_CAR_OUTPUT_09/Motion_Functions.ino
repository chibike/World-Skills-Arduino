void forward()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(pwm, drivePower);
  digitalWrite(stb, HIGH);
}

void backward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwm, drivePower);
  digitalWrite(stb, HIGH);
}

void stop()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(pwm, 0);
  digitalWrite(stb, HIGH);
}

void standby()
{
  digitalWrite(stb, LOW);
}

void printString(char* str, int n)
{
  Serial.print(str);
  Serial.println(n);
}

bool onTime(long lastUpdateTime, int waitTime)
{
  if(millis()-lastUpdateTime >= waitTime) return HIGH;
  return LOW;
}

void forwardDistance(uint16_t distance)
{
  EXECUTING_COMMAND_STATE = HIGH;
  
  leftDisplacement = 0;
  rightDisplacement = 0;
  float endDistance = ((float)(leftDisplacement + rightDisplacement)/2) + distance;
  float distDiff;
  float speedDiff;
  int8_t correctSteerDirection = 0;
  const int correctSteer = 7;
  const int currentHeading = steeringAngle;
  
  while(endDistance > AVERAGE_DISPLACEMENT)
  {
    forward();
    distDiff = rightDisplacement - leftDisplacement;

    if(correctSteerDirection == 0 && leftSpeed > AVERAGE_SPEED)
    {
      correctSteerDirection = -1;
    }
    else if(correctSteerDirection == 0 && rightSpeed > AVERAGE_SPEED)
    {
      correctSteerDirection = +1;
    }
    
    if( correctSteerDirection < 0 && distDiff < 0 )
    {
      setHeading(currentHeading-correctSteer);
    }
    else if( correctSteerDirection < 0 && distDiff == 0 )
    {
      setHeading(currentHeading);
      correctSteerDirection = 0;
    }
    else if( correctSteerDirection < 0 && distDiff > 0 )
    {
      setHeading(currentHeading+correctSteer);
      correctSteerDirection = 1;
    }
    else if( correctSteerDirection > 0 && distDiff > 0 )
    {
      setHeading(currentHeading+correctSteer);
    }
    else if( correctSteerDirection > 0 && distDiff == 0 )
    {
      setHeading(currentHeading);
      correctSteerDirection = 0;
    }
    else if( correctSteerDirection > 0 && distDiff < 0 )
    {
      setHeading(currentHeading-correctSteer);
      correctSteerDirection = -1;
    }
    
    pause();
  }
  stop();
  setHeading(currentHeading);

  EXECUTING_COMMAND_STATE = LOW;
}


void backwardDistance(uint16_t distance)
{
  EXECUTING_COMMAND_STATE = HIGH;
  
  leftDisplacement = 0;
  rightDisplacement = 0;
  float endDistance = ((float)(leftDisplacement + rightDisplacement)/2) - distance;
  float distDiff;
  float speedDiff;
  int8_t correctSteerDirection = 0;
  const int correctSteer = 7;
  const int currentHeading = steeringAngle;
  
  while(endDistance < AVERAGE_DISPLACEMENT)
  {
    backward();
    distDiff = rightDisplacement - leftDisplacement;

    if(correctSteerDirection == 0 && leftSpeed < AVERAGE_SPEED)
    {
      correctSteerDirection = -1;
    }
    else if(correctSteerDirection == 0 && rightSpeed < AVERAGE_SPEED)
    {
      correctSteerDirection = +1;
    }
    
    if( correctSteerDirection > 0 && distDiff > 0 )
    {
      setHeading(currentHeading-correctSteer);
    }
    else if( correctSteerDirection < 0 && distDiff == 0 )
    {
      setHeading(currentHeading);
      correctSteerDirection = 0;
    }
    else if( correctSteerDirection > 0 && distDiff < 0 )
    {
      setHeading(currentHeading-correctSteer);
      correctSteerDirection = -1;
    }
    else if( correctSteerDirection < 0 && distDiff < 0 )
    {
      setHeading(currentHeading+correctSteer);
    }
    else if( correctSteerDirection < 0 && distDiff == 0 )
    {
      setHeading(currentHeading);
      correctSteerDirection = 0;
    }
    else if( correctSteerDirection < 0 && distDiff < 0 )
    {
      setHeading(currentHeading+correctSteer);
      correctSteerDirection = +1;
    }
    
    pause();
  }
  stop();
  setHeading(currentHeading);

  EXECUTING_COMMAND_STATE = LOW;
}

void backwardDistance2(uint8_t distance)
{
  EXECUTING_COMMAND_STATE = HIGH;
  
  leftDisplacement = 0;
  rightDisplacement = 0;
  float endDistance = ((float)(leftDisplacement + rightDisplacement)/2) - distance;
  float distDiff;
  float speedDiff;

  static long timer = millis();
  while(endDistance < (float)AVERAGE_DISPLACEMENT)
  {
    backward();
    distDiff = rightDisplacement - leftDisplacement;
    speedDiff = rightSpeed - leftSpeed;
    printString("DistDiff: ",distDiff);
    printString("SpeedDiff: ",speedDiff);
    printString("Current Angle: ",steeringAngle);
    if(onTime(timer, 160))
    {
      if(distDiff > 0 )
      {
        setHeading(steeringAngle-3);
      }
      else if(distDiff < 0)
      {
        setHeading(steeringAngle+3);
      }
      timer = millis();
    }
    pause();
  }
  stop();

  EXECUTING_COMMAND_STATE = LOW;
}


void turnForwardDistance(int angle, int distance)
{
  EXECUTING_COMMAND_STATE = HIGH;
  
  float endDistance = ((float)(leftDisplacement + rightDisplacement)/2) + distance;

  while(endDistance > (float)AVERAGE_DISPLACEMENT)
  {
    setHeading(angle);
    forward();
    pause();
  }
  stop();

  EXECUTING_COMMAND_STATE = LOW;
}

void turnBackwardDistance(int angle, int distance)
{
  EXECUTING_COMMAND_STATE = HIGH;
  
  setHeading(angle);
  float endDistance = ((float)(leftDisplacement + rightDisplacement)/2) - distance;

  while(endDistance < (float)AVERAGE_DISPLACEMENT)
  {
    backward();
    pause();
  }
  stop();

  EXECUTING_COMMAND_STATE = LOW;
}


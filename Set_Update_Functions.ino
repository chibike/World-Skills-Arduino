void updateRegister()
{
  SPI.transfer(currentRegisterState);
  digitalWrite(latch, HIGH);
  digitalWrite(latch, LOW);
}

void setHeading(int16_t heading)
{
  steeringAngle = constrain(heading, -50, 50);
  printString("Angle = ",steeringAngle);
  steering.write((-1*steeringAngle)+110);
}

void setDrivePower(uint8_t power)
{
  drivePower = power;
}

void toggleBit(byte registerByte, uint8_t n)
{
  if(currentRegisterState & registerByte >> n == 0)
  {
    currentRegisterState |= registerByte;
  }
  else
  {
    currentRegisterState &= ~(registerByte);
  }
  updateRegister();
}

void flashAllLights()
{
  currentBlinkBits &= ~ALL_LIGHTS;
  allLightsFlashTimer = 6;
}

void flashHeadLights()
{
  currentBlinkBits &= ~HEAD_LIGHTS;
  headLightsFlashTimer = 6;
}

void headlamp()
{
  headLamp = !headLamp;
}

void flashHorn()
{
  hornFlashTimer = 6;
}

void pause()
{
  while(onPause)
  {
    Serial.println("onPause");
    stop();
  }
}


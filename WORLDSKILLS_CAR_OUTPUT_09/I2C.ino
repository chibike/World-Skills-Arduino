void receiveEvent(int n)
{
  int buffer = Wire.read();
  char burntData;

  switch(buffer)
  {
    case STOP_CMD:
      stop();
      break;
    case FWD:
      forward();
      break;
    case FWD_DIST:
      currentFunctionIndex = FWD_DIST;
      variable_1 |= (uint8_t)Wire.read();
      variable_1 = (variable_1 << 8) & 0xff00;
      variable_1 |= (uint8_t)Wire.read();
      break;
    case BWD:
      backward();
      break;
    case BWD_DIST:
      currentFunctionIndex = BWD_DIST;
      variable_1 |= (uint8_t)Wire.read();
      variable_1 = (variable_1 << 8) & 0xff00;
      variable_1 |= (uint8_t)Wire.read();
      break;
    case STB:
      standby();
      break;
    case ST_HD:
      buffer = Wire.read();
      setHeading(buffer-50);
      break;
    case ST_PWR:
      buffer = Wire.read();
      setDrivePower(buffer);
      break;
    case FL_HRN:
      flashHorn();
      break;
    case FL_HDL:
      flashHeadLights();
      break;
    case HDL:
      headlamp();
      break;
    case TRN_FWD:
      currentFunctionIndex = TRN_FWD;
      variable_1 = Wire.read();
      variable_1 |= (uint8_t)Wire.read();
      variable_1 -= 50;
      variable_2 = (variable_1 << 8) & 0xff00;
      variable_2 |= (uint8_t)Wire.read();
      break;
    case TRN_BWD:
      currentFunctionIndex = TRN_BWD;
      variable_1 = Wire.read()-50;
      variable_1 |= (uint8_t)Wire.read();
      variable_1 -= 50;
      variable_2 = (variable_1 << 8) & 0xff00;
      variable_2 |= (uint8_t)Wire.read();
      break;
    case NONE_CMD:
      break;
    case PAUSE:
      onPause = HIGH;
      currentFunctionIndex = PAUSE;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case PLAY:
      onPause = LOW;
      currentFunctionIndex = PAUSE;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case AVD_OBS:
      avoidObstacle = HIGH;
      break;
    
    default:
      stop();
  }
}


void requestEvent()
{
  if(EXECUTING_COMMAND_STATE == HIGH)
  {
    Wire.write(255);
  }
  else
  {
    //Serial.println("OKAY");
    Wire.write(0);
  }
}


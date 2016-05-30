#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

/* FUNCTIONS    COMMAND ID*/
//#define STOP        0x00
#define STOP_CMD    0x00
#define FWD         0x01
#define FWD_DIST    0x02
#define BWD         0x03
#define BWD_DIST    0x04
#define STB         0x05
#define ST_HD       0x06
#define ST_THD      0x07
#define ST_PWR      0x08
#define LT_IND      0x09
#define RT_IND      0x10
#define FL_AL       0x11
#define FL_HDL      0x12
#define FL_HRN      0x13
#define PAUSE       0x14
#define PLAY        0x15
#define FOLLOW_LINE 0x16
#define DWN_MAP     0x17
#define END_MAP     0x18
#define HDL         0x19
#define TRN_FWD     0x20
#define TRN_BWD     0x21
#define AVD_OBS     0x23
#define NONE_CMD    0x50

#define HEAD_LIGHTS 0x01
#define FAN 0x80
#define NONE 0x00
#define ALL_LIGHTS HEAD_LIGHTS

#define AVERAGE_DISPLACEMENT ((float)(leftDisplacement + rightDisplacement)/2)
#define AVERAGE_SPEED ((float)(leftSpeed + rightSpeed)/2)

Servo steering;

const int clk = 13;
const int data = 11;
const int latch = 8;

const int in1 = 12;
const int in2 = 10;
const int pwm = 6;
const int leftChA = 3;
const int leftChB = 7;
const int rightChA = 2;
const int rightChB = A1;
const int stb = 4;
const int lineSensorPin = A2;

const int steeringPin = 5;
const int hornPin = 9;

const int ownAddress = 8;

volatile float leftDisplacement = 0;
volatile float rightDisplacement = 0;
volatile float leftSpeed = 0;
volatile float rightSpeed = 0;
volatile uint8_t headLightsFlashTimer = 0;
volatile uint8_t allLightsFlashTimer = 0;
volatile uint8_t hornFlashTimer = 0;
volatile uint16_t fanTimer = 0;
volatile bool onPause = LOW;
volatile int variable_1 = 0;
volatile int variable_2 = 0;

const int fanOnTime = 180;
const int fanOffTime = 100;

uint8_t drivePower = 0;
int16_t steeringAngle = 0;
uint8_t driveSpeed = 25;
bool STOP = LOW;
bool headLamp = LOW;
bool EXECUTING_COMMAND_STATE = LOW;
bool avoidObstacle = LOW;
byte currentRegisterState = HEAD_LIGHTS;
byte currentBlinkBits = NONE;
int currentFunctionIndex = NONE_CMD;

void setup()
{
  EXECUTING_COMMAND_STATE = HIGH;
  initializePins();
  
  {
    cli();
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 254; //980.45 times a seconds
    TCCR2A |= (1<<WGM21);
    TCCR2B |= (1<<CS22);
    TIMSK2 |= (1<<OCIE2A);
    sei();
  }

  startRountine();
  //Serial.begin(9600); 
  EXECUTING_COMMAND_STATE = HIGH;
}

void loop()
{
  EXECUTING_COMMAND_STATE = LOW;
  
  switch(currentFunctionIndex)
  {
    case FWD_DIST:
      forwardDistance(variable_1);
      currentFunctionIndex = NONE_CMD;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case BWD_DIST:
      backwardDistance(variable_1);
      currentFunctionIndex = NONE_CMD;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case TRN_FWD:
      turnForwardDistance(variable_1, variable_2);
      currentFunctionIndex = NONE_CMD;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case TRN_BWD:
      turnBackwardDistance(variable_1, variable_2);
      currentFunctionIndex = NONE_CMD;
      variable_1 = 0;
      variable_2 = 0;
      break;
    case PLAY:
    case PAUSE:
      pause();
      currentFunctionIndex = NONE_CMD;
      variable_1 = 0;
      variable_2 = 0;
      break;
  }
}

void initializePins()
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(stb, OUTPUT);

  pinMode(leftChA, INPUT_PULLUP);
  pinMode(leftChB, INPUT_PULLUP);
  pinMode(rightChA, INPUT_PULLUP);
  pinMode(rightChB, INPUT_PULLUP);

  pinMode(clk, OUTPUT);
  pinMode(data, OUTPUT);
  pinMode(latch, OUTPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(lineSensorPin, INPUT_PULLUP);
  
  attachInterrupt(0, RIGHT_ISR, CHANGE);
  attachInterrupt(1, LEFT_ISR, CHANGE);

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  Wire.begin(ownAddress);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  digitalWrite(hornPin, LOW);
  steering.attach(steeringPin);
}


void startRountine()
{
  stop();
  flashAllLights();
  //flashHorn();
  setHeading(0);
  setDrivePower(150);
}

void RIGHT_ISR()
{
  if(digitalRead(rightChB) == digitalRead(rightChA))
  {
    rightDisplacement += 0.29;
  }
  else
  {
    rightDisplacement -= 0.29;
  }
}


void LEFT_ISR()
{
  if(digitalRead(leftChB) == digitalRead(leftChA))
  {
    leftDisplacement -= 0.29;
  }
  else
  {
    leftDisplacement += 0.29;
  }
}


ISR(TIMER2_COMPA_vect)
{
  static volatile int counter = 0;
  static volatile float lastLeftDisplacement = leftDisplacement;
  static volatile float lastRightDisplacement = rightSpeed;
  static volatile bool fanState = LOW;
  static volatile bool headLightsLastFlashState = LOW;
  static volatile bool allLightsLastFlashState = LOW;
  static volatile bool hornLastFlashState = LOW;
  static volatile bool lastBlinkState = LOW;
  
  if(counter++ >= 980)
  {
    leftSpeed = (leftDisplacement - lastLeftDisplacement);
    rightSpeed = (rightDisplacement - lastRightDisplacement);

    lastLeftDisplacement = leftDisplacement;
    lastRightDisplacement = rightDisplacement;
    /*
    if(AVERAGE_SPEED > abs(driveSpeed))
    {
      setDrivePower(drivePower-2);
    }
    else if(AVERAGE_SPEED < abs(driveSpeed))
    {
      setDrivePower(drivePower+2);
    }
    */

    fanTimer++;
    if(fanTimer >= fanOffTime && fanState == LOW)
    {
      fanTimer = 0;
      fanState = HIGH;
      currentRegisterState |= FAN;
    }
    else if(fanTimer >= fanOnTime && fanState == HIGH)
    {
      fanTimer = 0;
      fanState = LOW;
      currentRegisterState &= ~FAN;
    }
    else
    {
      //pass
    }

    if(lastBlinkState == LOW)
    {
      currentRegisterState |= currentBlinkBits;
      lastBlinkState = HIGH;
    }
    else
    {
      currentRegisterState &= ~currentBlinkBits;
    }

    if(allLightsFlashTimer > 0)
    {
      allLightsFlashTimer -= 1;
      if(allLightsLastFlashState == LOW)
      {
        currentRegisterState |= ALL_LIGHTS;
        allLightsLastFlashState = HIGH;
      }
      else
      {
        currentRegisterState &= ~ALL_LIGHTS;
        allLightsLastFlashState = LOW;
      }
    }
    else if(allLightsFlashTimer <= 0 && allLightsLastFlashState == HIGH)
    {
      currentRegisterState &= ~ALL_LIGHTS;
      allLightsLastFlashState = LOW;
    }
    else if(headLightsFlashTimer > 0)
    {
      headLightsFlashTimer -= 1;
      if(headLightsLastFlashState == LOW)
      {
        currentRegisterState  |= HEAD_LIGHTS;
        headLightsLastFlashState = HIGH;
      }
      else
      {
        currentRegisterState &= ~HEAD_LIGHTS;
        headLightsLastFlashState = LOW;
      }
    }
    else if(headLamp == HIGH)
    {
      currentRegisterState |= HEAD_LIGHTS;
      headLightsLastFlashState = HIGH;
    }
    else if(headLightsFlashTimer <= 0 && headLightsLastFlashState == HIGH)
    {
      currentRegisterState &= ~HEAD_LIGHTS;
      headLightsLastFlashState = LOW;
    }

    if(hornFlashTimer > 0)
    {
      hornFlashTimer -= 1;
      if(hornLastFlashState == LOW)
      {
        digitalWrite(hornPin, HIGH);
        hornLastFlashState = HIGH;
      }
      else
      {
        digitalWrite(hornPin, LOW);
        hornLastFlashState = LOW;
      }
    }

    updateRegister();
    counter = 0;
  }
}


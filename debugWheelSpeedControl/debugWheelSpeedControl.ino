#include <motorControl2342L.h>
#include <Arduino.h>

#define PI 3.14159
unsigned long savleftWheelInterrupt;
//-- General Robot parameters --
//int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
//int iRightMotorMaxrpm = iLeftMotorMaxrpm; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iRightMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int fMaxrpmAdjustment;  // will be used to compansate speed difference betweeen motors
int iLeftWheelDiameter = 65; //(in mm - used to measure robot moves)
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
int iLeftMotorDemultiplierPercent = 100; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
int iRightMotorDemultiplierPercent = iLeftMotorDemultiplierPercent; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
int iLeftTractionDistPerRev = 2 * PI * iLeftWheelDiameter / 2 * iLeftMotorDemultiplierPercent / 100;
int iRightTractionDistPerRev = 2 * PI * iRightWheelDiameter / 2 * iRightMotorDemultiplierPercent / 100;

//-- left Motor connection --
#define leftMotorENA 6 // Arduino pin must be PWM
#define leftMotorIN1 26 // arduino pin for rotation control
#define leftMotorIN2 27  // arduino pin for rotation control
#define wheelSpeedLeftPin 19   // pin
#define leftWheelEncoderHoles 8
//-- right Motor connection --
#define rightMotorENB 3 // Arduino pin must be PWM
#define rightMotorIN3 25 // arduino pin for rotation control
#define rightMotorIN4 24  // arduino pin for rotation control
#define wheelSpeedRightPin 18  // pin 
#define rightWheelEncoderHoles leftWheelEncoderHoles
//boolean bClockwise = true; //Used to turn the motor clockwise or counterclockwise
boolean bForward = true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
boolean firstCycle = true;
unsigned long iDistance = 50000; // mm
int iSpeedLeft = 408; // mm/s 408.4 maxi
int iSpeedRight = 408; // mm/s 408.4 maxi
unsigned long iLeftCentiRevolutions;
unsigned long iRightCentiRevolutions;
unsigned long restartedDelay;
unsigned long prevLeftWheelIntTime;
unsigned long prevRightWheelIntTime;
unsigned long delayCheckSpeed;
unsigned long delayPrintSpeed;
int iLeftRevSpeed;
int iRightRevSpeed;
float avgRightWheelSpeed = 0;
float avgLeftWheelSpeed = 0;
#define sizeOfLeftRev 4
#define sizeOfRightRev 8
unsigned int instantLeftWheelRevSpeed[sizeOfLeftRev];
unsigned int instantRightWheelRevSpeed[sizeOfRightRev];

uint8_t leftWheelSpeedCount = 0x00;
uint8_t rightWheelSpeedCount = 0x00;
unsigned long prevLeftWheelInterrupt = 0;
volatile unsigned long leftWheelInterrupt = 0;
volatile unsigned long copyleftWheelInterrupt = 0;
volatile unsigned long leftTimeInt = 0;
unsigned long prevRightWheelInterrupt = 0;
volatile unsigned long rightWheelInterrupt = 0;
unsigned long saveLeftWheelInterrupt = 0;
unsigned long saveRightWheelInterrupt = 0;
unsigned long delayInterrupt;
boolean motorsOn = false;
boolean interLeftOn = false;
boolean pause = false;
#define iLeftSlowPMW 30 // PMW value to slowdown motor at the end of the run
#define iRightSlowPMW iLeftSlowPMW   // 
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPMW);
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPMW);

void setup() {
  Serial.begin(38400);
  Serial.print("Moving ");
  Serial.print(iDistance);
  Serial.print(" mm at ");
  Serial.print(iSpeedLeft);
  Serial.println(" mm/s means");
  ComputerMotorsRevolutionsAndrpm(iDistance, iSpeedLeft, iDistance, iSpeedRight);
  Serial.print("Moving during ");
  Serial.print(iLeftCentiRevolutions);
  Serial.print(" centi-revolutions at ");
  Serial.print(iLeftRevSpeed);
  Serial.println(" turns per minute");
  fMaxrpmAdjustment = 90;
  Serial.println(fMaxrpmAdjustment);
  Serial.println(sizeOfLeftRev);
  pinMode(wheelSpeedLeftPin, INPUT);
  pinMode(wheelSpeedRightPin, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, 0);
  Serial.print("int n:");
  Serial.println(digitalPinToInterrupt(wheelSpeedLeftPin));
  //  startMotors();
}

void loop() {
  // Serial.print("millis: ");
  //  Serial.println(millis());
  /*
  if (millis() - delayInterrupt > 20 && interLeftOn == false)
  {
    Serial.println("att");
    attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, FALLING);
    interLeftOn = true;
    delayInterrupt = millis();
  }
  */
  /*
  if (copyleftWheelInterrupt + 1 != leftWheelInterrupt && leftWheelInterrupt!=0)
  {
    digitalWrite(46, 1);
    Serial.print("incoh ");
    Serial.print (copyleftWheelInterrupt);
        Serial.print(" ");
    Serial.print (leftWheelInterrupt);
  }
  */
  if (pause == false)
  {
    if (leftWheelInterrupt - savleftWheelInterrupt > 1)
    {
      Serial.print("prev:");
      Serial.print(savleftWheelInterrupt);
      Serial.print("curr:");
      Serial.print(leftWheelInterrupt);
      Serial.print("millis:");
      Serial.println(millis());
    }
    savleftWheelInterrupt = leftWheelInterrupt;
    if (millis() - delayCheckSpeed > 250 && motorsOn == true)
    {
      SpeedRightWheel();
      SpeedLeftWheel();

      delayCheckSpeed = millis();
      avgLeftWheelSpeed = 0;
      for (int i = 0; i < sizeOfLeftRev; i++)
      {
        avgLeftWheelSpeed = instantLeftWheelRevSpeed[i] + avgLeftWheelSpeed;
      }
      avgLeftWheelSpeed = avgLeftWheelSpeed / sizeOfLeftRev;
      avgRightWheelSpeed = 0;
      for (int i = 0; i < sizeOfRightRev; i++)
      {
        avgRightWheelSpeed = avgRightWheelSpeed + instantRightWheelRevSpeed[i]  ;
      }
      avgRightWheelSpeed = avgRightWheelSpeed / sizeOfRightRev;
    }

    if (millis() - delayPrintSpeed > 500)
    {
      if (avgLeftWheelSpeed > 500)
      {
        //        digitalWrite(46, 1);
        Serial.print("to high ");
        Serial.println(millis());
        leftMotor.StopMotor();
        rightMotor.StopMotor();
        pause = true;
      }
      Serial.print("leftAvpSpeed:");
      Serial.print(avgLeftWheelSpeed);
      Serial.print(" rightAvpSpeed:");
      Serial.println(avgRightWheelSpeed);
      delayPrintSpeed = millis();
      Serial.print(leftWheelInterrupt );
      Serial.print(" ");
      Serial.print(rightWheelInterrupt);
      Serial.print(" ");
      Serial.println(millis());
    }


    int iLeftFeedBack = leftMotor.CheckMotor( (avgLeftWheelSpeed * iLeftMotorDemultiplierPercent) / 100, ((leftWheelInterrupt * 100) / leftWheelEncoderHoles) * iLeftMotorDemultiplierPercent / 100);
    int iRightFeedBack = rightMotor.CheckMotor((avgRightWheelSpeed * iRightMotorDemultiplierPercent) / 100  , ((rightWheelInterrupt * 100) / rightWheelEncoderHoles) * iRightMotorDemultiplierPercent / 100 );
    if (iLeftFeedBack == -2)
    {
      Serial.println("leftMotor Pb");

    }
    if (iRightFeedBack == -2)
    {
      Serial.println("RightMotor Pb");
    }
    if (iLeftFeedBack <= 0 || iRightFeedBack <= 0)
    {
      //   motorsOn = false;
    }
    // Serial.print(iLeftRpm);
    //  Serial.print(" - ");
    //  Serial.println(iRightRpm);

    if (firstCycle == true)
    {
      startMotors();
      Serial.println("start");
      firstCycle = false;
    }
    if (millis() - restartedDelay >= 3000000 & iDistance != 0)
    {
      startMotors();
      restartedDelay = millis();
      Serial.println("re-start");
    }
    // delay(100);
  }
}
void startMotors() {
  StartLeftWheelSpeedControl();   // pour test
  // StartRightWheelSpeedControl();   // pour test
  leftMotor.TurnMotor(bLeftClockwise, iLeftCentiRevolutions, iLeftRevSpeed);
  //  rightMotor.TurnMotor(bRightClockwise, iRightCentiRevolutions, iRightRevSpeed);
  motorsOn = true;
}

void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, int iLeftSpeed, unsigned long iRightDistance, int iRightSpeed)
{
  iLeftCentiRevolutions = iLeftDistance * 100 / iLeftTractionDistPerRev ; // Centi-revolutions
  iRightCentiRevolutions = (iRightDistance * 100 / iRightTractionDistPerRev) ; // ms
  iLeftRevSpeed = (iLeftSpeed * 60) / iLeftTractionDistPerRev; // revolutions per minute
  iRightRevSpeed = (iRightSpeed * 60 ) / iRightTractionDistPerRev; // revolutions per minute
  /*
  Serial.println(iLeftRevSpeed);
  Serial.println(iLeftTractionDistPerRev);
  Serial.println(iLeftCentiRevolutions);

  Serial.println(iRightRevSpeed);
  Serial.println(iRightTractionDistPerRev);
  Serial.println(iRightCentiRevolutions);
  */
}
void StartLeftWheelSpeedControl()
{

  for (int i = 0; i < sizeOfLeftRev; i++)
  {
    instantLeftWheelRevSpeed[i] = 0; // init avec expected speed
  }
  attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, CHANGE);
  leftWheelInterrupt = 0;
  leftTimeInt = 0;
}
void StopLeftWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin));
}
void LeftWheelCount()
{
 
  if (leftTimeInt != 0 && micros() - leftTimeInt < 1000)
  {
    digitalWrite(8, 1);
    Serial.print("m:");
    Serial.print(micros());
    Serial.print(" ");
    Serial.print(leftTimeInt);
    Serial.print(" ");
    Serial.println(leftWheelInterrupt);
      //   noInterrupts();
      // detachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin));
      //    unsigned long savleftWheelInterrupt=leftWheelInterrupt;
  //    copyleftWheelInterrupt = leftWheelInterrupt;
   //   leftWheelInterrupt = leftWheelInterrupt + 1;
      //   interrupts();

      //   attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, CHANGE);
      // interLeftOn = false;
    }
    else
  {
//   digitalWrite(8, 0);
  }
 
  //      digitalWrite(44, digitalRead(wheelSpeedLeftPin));
        copyleftWheelInterrupt = leftWheelInterrupt;
      leftWheelInterrupt = leftWheelInterrupt + 1;
  leftTimeInt = micros();
}
void StartRightWheelSpeedControl()
{

  for (int i = 0; i < sizeOfRightRev; i++)
  {
    instantRightWheelRevSpeed[i] = 0; // init avec expected speed
  }
  attachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin), RightWheelCount, FALLING);
  rightWheelInterrupt = 0;
}
void StopRightWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin));
}
void RightWheelCount()
{
  noInterrupts();
  rightWheelInterrupt++;
  interrupts();
}
void SpeedRightWheel()
{
  unsigned int deltaTime = millis() - prevRightWheelIntTime;
  prevRightWheelIntTime = millis();
  unsigned int deltaEncoder = rightWheelInterrupt - prevRightWheelInterrupt;
  prevRightWheelInterrupt = rightWheelInterrupt;
  //  Serial.print("right wheel rpm:");
  instantRightWheelRevSpeed[rightWheelSpeedCount % sizeOfRightRev] = 60 * (float (deltaEncoder * 1000 / deltaTime) / rightWheelEncoderHoles);
  //  Serial.println(instantRightWheelRevSpeed[rightWheelSpeedCount % sizeof(instantRightWheelRevSpeed) / 2]);
  rightWheelSpeedCount++;
}
void SpeedLeftWheel()
{
  unsigned int deltaTime = millis() - prevLeftWheelIntTime;
  prevLeftWheelIntTime = millis();
  unsigned int deltaEncoder = leftWheelInterrupt - prevLeftWheelInterrupt;
  prevLeftWheelInterrupt = leftWheelInterrupt;
  //  Serial.print("left wheel rpm:");
  instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeOfLeftRev] = 60 * (float (deltaEncoder * 1000 / deltaTime) / leftWheelEncoderHoles);
  //  Serial.println(instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeof(instantLeftWheelRevSpeed) / 2]);
  //    Serial.println(leftWheelSpeedCount % sizeOfLeftRev);
  leftWheelSpeedCount++;
}


void CheckMoveSynchronisation()
{
  float deltaMove = ((leftWheelInterrupt - rightWheelInterrupt) ) ;

  if ( abs(deltaMove) > 4)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    Serial.print("move synchro pb:");
    Serial.println(deltaMove);


  }
}

void stopWheelControl()
{
  boolean last = true;
  StopRightWheelSpeedControl();
  StopLeftWheelSpeedControl();

}




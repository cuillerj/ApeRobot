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
#define leftMotorENA 12 // Arduino pin must be PWM  use timer 3
#define leftMotorIN1 26 // arduino pin for rotation control
#define leftMotorIN2 27  // arduino pin for rotation control
#define leftMotorId 0    // used to identify left motor inside the code
#define wheelSpeedLeftPin 19   // pin
#define leftWheelEncoderHoles 8
//-- right Motor connection --
#define rightMotorENB 11 // Arduino pin must be PWM use timer 3
#define rightMotorIN3 25 // arduino pin for rotation control
#define rightMotorIN4 24  // arduino pin for rotation control
#define rightMotorId 1    // used to identify right motor inside the code
#define wheelSpeedRightPin 18  // pin 
#define rightWheelEncoderHoles leftWheelEncoderHoles
//boolean bClockwise = true; //Used to turn the motor clockwise or counterclockwise
boolean bForward = true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
boolean firstCycle = true;
unsigned long iDistance = 50000; // mm
int iSpeedLeft = 204; // mm/s 408.4 maxi
int iSpeedRight = 204; // mm/s 408.4 maxi
unsigned long iLeftCentiRevolutions=8;
unsigned long iRightCentiRevolutions=8;
unsigned long restartedDelay;
unsigned long prevLeftWheelIntTime;
unsigned long prevRightWheelIntTime;
unsigned long delayCheckSpeed;
unsigned long delayPrintSpeed;
int iLeftRevSpeed=60;
int iRightRevSpeed=60;
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
  pinMode(46, INPUT);
  digitalWrite(46, 0);
  Serial.print("int n:");
  Serial.println(digitalPinToInterrupt(wheelSpeedLeftPin));
  //  startMotors();
}

void loop() {
  startMotors();
  delay(10000);
}
void startMotors() {

  leftMotor.RunMotor(bLeftClockwise, 255);
  rightMotor.RunMotor(bRightClockwise, 160);
  motorsOn = true;
}





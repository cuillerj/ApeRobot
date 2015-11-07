#include <motorControl2342L.h>
#include <Arduino.h>

#define PI 3.14159

//-- General Robot parameters --
int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iRightMotorMaxrpm = iLeftMotorMaxrpm; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iLeftWheelDiameter = 65; //(in mm - used to measure robot moves)
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
int iLeftMotorDemultiplierPercent = 100; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
int iRightMotorDemultiplierPercent = iLeftMotorDemultiplierPercent; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
int iLeftTractionDistPerRev = 2 * PI * iLeftWheelDiameter / 2 * iLeftMotorDemultiplierPercent / 100;
int iRightTractionDistPerRev = 2 * PI * iRightWheelDiameter / 2 * iRightMotorDemultiplierPercent / 100;

//-- left Motor connection --
int leftMotorENA = 3; //Connecté à Arduino pin 3(sortie pwm)
int leftMotorIN1 = 2; //Connecté à Arduino pin 2
int leftMotorIN2 = 4; //Connecté à Arduino pin 3

//-- right Motor connection --
int rightMotorENB = 6; //Connecté à Arduino pin 6(Sortie pwm)
int rightMotorIN3 = 7; //Connecté à Arduino pin 4
int rightMotorIN4 = 8; //Connecté à Arduino pin 7

//boolean bClockwise = true; //Used to turn the motor clockwise or counterclockwise
boolean bForward = true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
boolean firstCycle = true;
unsigned long iDistance = 1000; // mm
int iSpeed = 409; // mm/s 408.4 maxi 
unsigned long iLeftCentiRevolutions;
unsigned long iRightCentiRevolutions;
unsigned long restartedDelay;
int iLeftRevSpeed;
int iRightRevSpeed;

Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm);
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm);

void setup() {
  Serial.begin(38400);
  Serial.print("Moving ");
  Serial.print(iDistance);
  Serial.print(" mm at ");
  Serial.print(iSpeed);
  Serial.println(" mm/s means");
  ComputerMotorsRevolutionsAndrpm(iDistance, iSpeed, iDistance, iSpeed);
  Serial.print("Moving during ");
  Serial.print(iLeftCentiRevolutions);
  Serial.print(" centi-revolutions at ");
  Serial.print(iLeftRevSpeed);
  Serial.println(" turns per minute");
  //  startMotors();
}

void loop() {
  // Serial.print("millis: ");
  //  Serial.println(millis());
 int iLeftRpm = leftMotor.CheckMotor();
int iRightRpm = rightMotor.CheckMotor();
 // Serial.print(iLeftRpm);
//  Serial.print(" - ");
//  Serial.println(iRightRpm);

  if (firstCycle == true)
  {
       startMotors();
    Serial.println("start");
    firstCycle = false;
  }
  if (millis() - restartedDelay >= 30000)
  {
      startMotors();
    restartedDelay = millis();
    Serial.println("re-start");
  }
  delay(100);
}

void startMotors() {
  leftMotor.TurnMotor(bLeftClockwise, iLeftCentiRevolutions, iLeftRevSpeed);
  rightMotor.TurnMotor(bRightClockwise, iRightCentiRevolutions, iRightRevSpeed);
}

void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, int iLeftSpeed, unsigned long iRightDistance, int iRightSpeed)
{
  iLeftCentiRevolutions = iLeftDistance / iLeftTractionDistPerRev * 100; // Centi-revolutions
  iRightCentiRevolutions = iRightDistance / iRightTractionDistPerRev * 100; // ms
  // Serial.println(iLeftSpeed);
  //  Serial.println(iLeftTractionDistPerRev);
  iLeftRevSpeed = iLeftSpeed * 60 / iLeftTractionDistPerRev; // revolutions per minute
  iRightRevSpeed = iRightSpeed * 60 / iRightTractionDistPerRev; // revolutions per minute
}

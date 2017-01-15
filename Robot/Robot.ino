/*
   v2 a nano added as sensor subsytem for gyroscope

   release notes

*/

String Version = "RobotV2";
// uncomment #define debug to get log on serial link
//#define debugScanOn true
#define debugMoveOn true
//#define debugObstacleOn true
//#define debugLocalizationOn true
//#define debugMagnetoOn true
//#define debugMotorsOn true
#define debugWheelControlOn true
#define wheelEncoderDebugOn true
//#define debugConnection true
//#define debugPowerOn true
//#define debugLoop true
//#define servoMotorDebugOn true
//#define gyroDebugOn true

#define debugGyroscopeOn true
#define debugGyroscopeL2On true
#define I2CSlaveMode true
#include <Servo.h>  // the servo library use timer 5 with atmega
#include <math.h>
#include <EEPROM.h>  // 
//#include <Stepper.h>
#include <SoftwareSerial.h>
#include <Wire.h>        // for accelerometer
#include <LSM303.h>     // for accelerometer
#include <EchoObstacleDetection.h>
#include <WheelControl.h>
#define IMU true
//#include <ApeRobotSensorSubsytemDefine.h>  // select to use a simple gyroscope
#include <BNO055SubsystemCommonDefine.h>     // or select to use IMU 
#include <NewPing.h>
Servo myservo;  // create servo object to control a servo

//-- comunication --
#include <SerialNetworkVariable.h>   // needed for communication with esp8266 gateway
#include <SerialNetworkVoid.h>  // needed for communication with esp8266 gateway

uint8_t PendingReqRef = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRef = 0xbf; // pending request (if 0x00 none)- copy fo retry
uint8_t PendingReqRefSerial = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRefSerial = 0xbf; // pending request (if 0x00 none)- copy fo retry
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
uint8_t trameNumber = 0;     // frame number to send
uint8_t lastAckTrameNumber = 0;  // last frame number acknowledged by the server
uint8_t pendingAckSerial = 0;    // flag waiting for acknowledged
int retryCount = 0;            // number of retry for sending

//-- General Robot parameters --
#include <ApeRobotCommonDefine.h>
uint8_t rebootPhase = 3;

float iLeftTractionDistPerRev =  (PI * iLeftWheelDiameter) ;
float iRightTractionDistPerRev = (PI * iRightWheelDiameter);
float coeffGlissementRotation = 1.;
#define rebootDuration 20000 // delay to completly start arduino
#define hornPin 49           // to activate horn

//-- power control --
int uRefMotorVoltage = 1200; // mVolt for maxRPM
//#define power1Pin 53  // power 1 servomoteur
#define power1Value A13  // 9v power for motors
#define power2Value A14  // 5v power for arduino mega
//#define power3Value A15  // 12V power for esp8266 and components
#define power1LowLimit 750   // minimum centi volt before warning
#define power2LowLimit 475   // minimum centi volt before warning
//#define power3LowLimit 750  // minimum centi volt before warning
int power1Mesurt = 0;   // current power1 value
int power2Mesurt = 0;   // current power2 value
#define encoderPower 48
//int power3Mesurt = 0;   // current power3 value

//-- left Motor connection --
#define leftMotorENA 12 // Arduino pin must be PWM  use timer 3
#define leftMotorIN1 26 // arduino pin for rotation control
#define leftMotorIN2 27  // arduino pin for rotation control
#define leftMotorId 0    // used to identify left motor inside the code
//-- right Motor connection --
#define rightMotorENB 11 // Arduino pin must be PWM use timer 3
#define rightMotorIN3 25 // arduino pin for rotation control
#define rightMotorIN4 24  // arduino pin for rotation control
#define rightMotorId 1    // used to identify right motor inside the code
//-- motors control --
#include <motorControl2342L.h>  // library for motors control
unsigned int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
unsigned int iRightMotorMaxrpm = iLeftMotorMaxrpm ; // (Value unknown so far - Maximum revolutions per minute for left motor)
//float fMaxrpmAdjustment;  // will be used to compensate speed difference betweeen motors
unsigned int leftMotorPWM = 230;       // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
#define iLeftSlowPWM 150        // PMW value to slowdown motor at the end of the run
#define leftRotatePWMRatio 0.8    // 
#define pendingLeftMotor 0      // define pendingAction bit used for left motor
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPWM); // define left Motor
unsigned int rightMotorPWM = 200;      // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
#define iRightSlowPWM 110  // PMW value to slowdown motor at the end of the run
//#define iRightRotatePWM 170   // PMW value
#define rightRotatePWMRatio 0.9    // 
#define pendingRightMotor 1    // define pendingAction bit used for right motor
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPWM); // define right Motor
float leftToRightDynamicAdjustRatio = 1.0;    // ratio used to compensate speed difference between the 2 motors rght PWM = left PWM x ratio
//-- wheel control --
#define wheelPinInterrupt 3    // used by sotfware interrupt when rotation reach threshold
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A7  // analog input right encoder
#define leftWheelId 0           // to identify left wheel Id 
#define rightWheelId 1         // to identify right wheel Id
unsigned int iLeftRevSpeed;              // instant left wheel speed
unsigned int iRightRevSpeed;             // instant right wheel speed
#define sizeOfLeftRev 8 // size of the array containing latest revolution wheel speed
#define sizeOfRightRev 8 // size of the array containing latest revolution wheel speed
unsigned long saveLeftWheelInterrupt = 0;           // copy of previous number of interrupts for the left encoder
unsigned long saveRightWheelInterrupt = 0;          // copy of previous number of interrupts for the right encoder
unsigned long pauseLeftWheelInterrupt = 0;           // copy of previous number of interrupts for the left encoder
unsigned long pauseRightWheelInterrupt = 0;          // copy of previous number of interrupts for the right encoder
unsigned int currentLeftWheelThreshold = 0;
unsigned int currentRightWheelThreshold = 0;
boolean pauseDirectionBackward;
long leftWheeelCumulative = 0;            // cumulative count of the left holes used for dynamic speed adjustment
long rightWheeelCumulative = 0;           // cumulative count of the right holes used for dynamic speed adjustment
unsigned long prevCheckLeftHoles = 0;      // copy of previous LeftHoles to check wheels not blocked
unsigned long prevCheckRightHoles = 0;      // copy of previous RightHoles to check wheels not blocked
// to adjust low and high value set leftWheelControlOn true, rotate left wheel manualy and read on serial the value with and wihtout hole
// must fit with the electonic characteristic
// use calibrate() to set the incoder high and low value
/*

   unsigned int leftIncoderHighValue = 300;  // define value above that signal is high
  unsigned int leftIncoderLowValue = 112;  // define value below that signal is low
  // to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
  unsigned int rightIncoderHighValue = 610; // define value above that signal is high
  unsigned int rightIncoderLowValue = 487;  // define value below that signal is low
*/
unsigned int leftIncoderHighValue = 650;  // define value mV above that signal is high
unsigned int leftIncoderLowValue = 250;  // define value mV below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
unsigned int rightIncoderHighValue = 640; // define value mV above that signal is high
unsigned int rightIncoderLowValue = 220;  // define value mV below that signal is low
//#define delayBetweenEncoderAnalogRead  750 //  micro second between analog read of wheel encoder level
#define delayMiniBetweenHoles  35  //  delay millis second between 2 encoder holes at the maximum speed
// create wheel control object
WheelControl Wheels(leftWheelEncoderHoles, leftIncoderHighValue, leftIncoderLowValue, leftAnalogEncoderInput,
                    rightWheelEncoderHoles, rightIncoderHighValue , rightIncoderLowValue, rightAnalogEncoderInput,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    wheelPinInterrupt, delayMiniBetweenHoles);
//-- move control --
# define bForward  true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
unsigned long iLeftCentiRevolutions;  // nmuber of done revolutions * 100
unsigned long iRightCentiRevolutions; // nmuber of done revolutions * 100
#define toDoGyroRotation 0      // rotation based on gyroscope to do
#define toDoScan 0    // toDo bit for scan request
#define toDoMove 1      // some move  to do
#define toDoRotation 2  // rotation to do
#define toDoStraight 3  // straight move to do
#define toDoBackward 4  // straight move to do is backward
#define toDoClockwise 5 // rotate clockwise
#define toDoAlign 6     // north aligning
#define toDoPingFB 7     // echo ping front back
#define toWait 0        // wait before moving
#define toEndPause 1     // could restart
#define toPause 2     // move to be temporaly paused
int gyroTargetRotation = 0; // expected rotation based on gyroscope
uint8_t gyroRotationRetry = 0;
#define maxGyroRotationRetry 30
int pendingStraight = 0;   // copy of straight distance to be done
int reqAng = 0;          // requested rotation
int reqMove = 0;         // requested move
uint8_t resumeCount = 0; // nb of echo check before resume move
int movePingInit;     // echo value mesured before moving
uint8_t rotationType = 0x00;
//-- accelerometer and magnetometer
// powered by arduino 3.3v
LSM303 compass;
unsigned long refAccX;
float X;
float Y = 0;
int  northOrientation = 0;
int saveNorthOrientation = 0; // last north orientation
int NOBeforeRotation = 0; // keep NO before rotation
int NOAfterRotation = 0;
int NOBeforeMoving = 0; // keep NO before moving straight
int NOAfterMoving = 0; // keep NO before moving straight
int targetAfterNORotation = 0;
// compass calibration to be made
#define compassMin1 -661
#define compassMin2 -1792
#define compassMin3 -3132
#define compassMax1 +1502
#define compassMax2 +497
#define compassMax3 -2560
/*
   gyroscope
*/
boolean gyroCalibrationOk = true;
uint8_t inputData[256];
uint8_t slaveAddress = robotI2CAddress;
long OutputRobotRequestPinTimer;
uint8_t outData[pollResponseLenght] = {slaveAddress, 0x00, 0x00, 0x00};
#define maxGyroscopeHeadings 100       // must be less than 255
int gyroscopeHeading[maxGyroscopeHeadings];
uint8_t gyroscopeHeadingIdx = 0x00;
uint8_t gyroUpToDate = 0x00;
uint8_t monitSubsystemStatus = 0x00;
int prevGyroRotation = 0;
uint8_t gyroInitRotationSens = 0x00;
//-- scan control --
#define servoPin 6    //  servo motor Pin (28)
#define echoFront 19  // arduino pin for mesuring echo delay for front 
#define echFrontId 0 //
#define trigFront  23     // arduino pin for trigerring front echo
#define echoBack  18   // arduino pin for mesuring echo delay for back 
#define trigBack  22    // arduino pin for trigerring back echo
#define echBacktId 1 //
#define nbPulse 15    // nb of servo positions for a 360Â° scan
#define echo3 false  // does not exit
#define echo3Alert false  // does not exit
#define echo4 false  // does not exit
#define echo4Alert false  // does not exit
#define echoPinInterrupt 2 // pin  dedicated to software usage 
boolean toDoEchoFront = true;   // to set echo front on when obstacle detection is running
boolean echoFrontAlertOn = true; // // to set echo front threshold on when obstacle detection is running
boolean toDoEchoBack = true;    // to set echo back on when obstacle detection is running
boolean echoBackAlertOn = true; // to set echo back threshold on when obstacle detection is running
float echoCycleDuration = 0.5;  // define cycle in second between 2 triggers when obstacle detection is running
EchoObstacleDetection echo(echoFront, trigFront, echoBack, trigBack, 0, 0, 0, 0, echoPinInterrupt);   // create obstacle detection object
boolean obstacleDetectionOn = true;
unsigned int obstacleDetectionCount = 0;
int numStep = 0;     // current scan step number
int nbSteps = 0;     // number of steps to be done for a scan
boolean switchFB = 0;  // switch front back scan
#define nbPulse 15    // nb of servo positions for a 360Â° scan
#define miniServoAngle 0    // corresponding to the lowest value of pulseValue
#define maxiServoAngle 180   // corresponding to the highest value of pulseValue
//int pulseValue[nbPulse] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165}; // corresponding to 180° split in 15 steps
//int pulseValue[nbPulse] = {0, 13, 26, 39, 52, 65, 78, 90, 103, 116, 129, 142, 155, 168, 180}; // corresponding to 180° split in 15 steps
int pulseValue[nbPulse] = {0, 13, 26, 38, 52, 65, 78, 90, 103, 116, 129, 142, 154, 166, 178}; // corresponding to 180° split in 15 steps
uint8_t shiftPulse = 0;             // eventually used to adjust servo motor reference position
float coefAngRef = PI / (pulseValue[14] - pulseValue[0]);  // angle value beetwen 2 pulses
uint8_t pulseNumber = 0;  // pointer to pulseValue array
int distFSav = 0;     // saved front echo distance in cm
int distBSav = 0;      // saved back echo distance in cm
int valAng;  // servo orientation
float AngleRadian;  // echo servo orientation in radian
float AngleDegre;    // echo servo orientation in degre
int scanOrientation = 0;    // define servo position before starting a 360 scan
uint8_t echoCurrent = 0; // used during move to determine which front or back is used
uint8_t trigCurrent = 0; // used during move to determine which front or back is used
boolean trigOn = false;  // flag for asynchronous echo to know if trigger has been activated
boolean trigBoth = false; // flag used to alternatively scan front or back
unsigned int scanNumber = 0;   // count number of scan retry for asynchronous echo
#define MAX_DISTANCE 400
NewPing pingFront(trigFront, echoFront, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing pingBack(trigBack, echoBack, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//-- timers --
unsigned long timeAppli; // cycle applicatif
unsigned long timeStatut;  // cycle envoi du statut au master
unsigned long timeNetwStat; // cycle affichage des stat RF433 sur serial
unsigned long timeRTC;  // cycle demande heure
unsigned long timeScanFront;  // cycle demande heure
unsigned long timeScanBack;  // cycle demande heure
unsigned long timeSendSecSerial;  // used to check for acknowledgment of secured frames
unsigned long timeReceiveSerial;  // used to regularly check for received message
unsigned long timePowerCheck;  // used to regularly check for power values
unsigned long prevLeftWheelIntTime;  // last time of the previuous left wheel interrupt
unsigned long prevRightWheelIntTime; // last time of the previuous right wheel interrupt
unsigned long delayCheckSpeed;       // when moving used to regularly compute speed of wheels
unsigned long delayCheckPosition;    // when moving used to regularly compute localization
unsigned long delayPrintSpeed;        // when moving used to regularly print speed for debug
unsigned long timeAffLed;        // used to regularly update LED status
unsigned long serialAliveTimer;  //  for serial link communication
unsigned long serialTimer;       //   for serial link communication
unsigned long checkObstacleTimer;     // to regurarly check obstacle when moving
unsigned long durationMaxEcho = 25000; // maximum scan echo duration en us
unsigned long timeSendInfo;     // to regurarly send information to th server
unsigned long delayAfterStopMotors;     // to avoid I2C perturbations
unsigned long timeBetweenOnOffObstacle;  // use to compute delay between obstacle detection sitch on off
unsigned long timeMotorStarted;          // set to time motors started
unsigned long timerHorn   ;          // set to time horn started
volatile unsigned long pauseSince   ;          // starting time of pause status
unsigned long timePingFB;             // used to delay sending end action after sending echo ping
unsigned long timeGyroRotation;
unsigned long timeSubsystemPolling;   // used for reguraly checking subsystem status
unsigned long timeUpdateNO;           // used for reguraly update north orientation
#if defined debugLoop
unsigned long timeLoop;  // cycle demande heure
#endif
#define delayBetween2Scan 1500  // delay between to scan steps - must take into account the transmission duration
#define delayBetweenScanFB 700  // delay between front and back scan of the same step
#define delayBetweenInfo 5000   // delay before sending new status to the server  
#define delayPowerCheck 5000    // delay before next checking of power
#define delayBeforeRestartAfterAbstacle 5000 // delay before taking into account obstacle disappearance
#define delayBetweenCheckPosition 200       // delay between 2 check position
#define delayBetween2NO 500          // delay between 2 NO update
#define maxPauseDuration 30000
//-- robot status & diagnostics
volatile uint8_t diagMotor = 0x00;   // bit 0 pb left motor, 1 right moto, 2 synchro motor
uint8_t diagPower = 0x00;   // bit 0 power1, 1 power2, 2 power3 0 Ok 1 Ko
uint8_t diagConnection = 0x03;   // bit 0 pending serial link bit 1 pending I2C
uint8_t diagRobot = 0xff;     // to be cleared after complete reboot
uint8_t toDo = 0x00;          // flag kind of move to do
uint8_t toDoDetail = 0x00;          // flag kind of move to do
volatile uint8_t waitFlag = 0xff;     // used to pause and waiting before starting

uint8_t currentMove = 0x00;   // flag kind of current move
uint8_t pendingAction = 0x00; // flag pending action to be started
uint8_t sendInfoSwitch = 0x00;  // flag to switch between different data to send
uint8_t saveCurrentMove = 0x00;   // flag last kind of current move
boolean moveForward;           // flag to indicate if requested move is forward or backward
uint8_t appStat = 0xff; // statut code application 00 active ff stopped 1er demi octet mouvement 2eme demi scan
uint8_t actStat = 0xff; // statut action en cours

//-- space localizarion --
float alpha = 0; // current orientation
float posX = 0;   // current x position
float posY = 0;   // current y position
float targetAlpha = 0; // orientation target position after moving
long targetX = 0;  // x target position after moving
long targetY = 0;// x target position after moving
float deltaPosX = 0;  // incremental x move
float deltaPosY = 0;  // incremental y move
float posRotationCenterX = posX - shiftEchoVsRotationCenter * cos(alpha*PI / 180);   // x position of the rotation center
float posRotationCenterY = posY - shiftEchoVsRotationCenter * sin(alpha*PI / 180);   // y position of the rotation center
float posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha*PI / 180);   // x position of the rotation center
float posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha*PI / 180);   // y position of the rotation center
uint8_t currentLocProb = 0;


//-- LED --
#define blueLed 50    // blue LED PIN
#define yellowLed 51    // yellow LED PIN
#define greenLed 52  // green LED PIN
#define redLed 53    // red LED PN
boolean blueLedOn =  true;  //  status of the LED on/off
boolean yellowLedOn = true; //  status of the LED on/off
boolean greenLedOn =  false; //  status of the LED on/off
boolean redLedOn = true;    //  status of the LED on/off

//
void setup() {
  Serial.begin(38400);            // for debugging log
  Serial2.begin(SpeedNetwSerial); // to communicate with the server through a serial Udp gateway
  Serial.println(Version);
  int addrEeprom = 0;
  byte valueEeprom;
  valueEeprom = EEPROM.read(addrEeprom);  // id of the arduino stored in eeprom
  Serial.print("ArduinoID=");
  Serial.println(valueEeprom, DEC);
  // **** define gpio mode
  Serial.println("reboot started");
  pinMode(trigFront, OUTPUT);
  pinMode(encoderPower, OUTPUT);
  digitalWrite(trigBack, LOW);
  pinMode(trigBack, OUTPUT);
  digitalWrite(trigFront, LOW);
  pinMode(echoFront, INPUT);
  pinMode(echoBack, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(wheelPinInterrupt, INPUT);
  pinMode(echoPinInterrupt, INPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(power1Value, INPUT);
  pinMode(power2Value, INPUT);
  pinMode(RobotOutputRobotRequestPin, OUTPUT);
  digitalWrite(RobotOutputRobotRequestPin, LOW);
  digitalWrite(hornPin, LOW);
  digitalWrite(blueLed, HIGH);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, HIGH);
  Serial.println("Start I2C");
#if defined(I2CSlaveMode)
  Wire.begin(slaveAddress);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.println("starting I2C slave mode");
  pinMode(RobotOutputRobotRequestPin, OUTPUT);
  digitalWrite(RobotOutputRobotRequestPin, LOW);
#else
  Wire.begin();
  compass.init();
  compass.enableDefault();
#endif
  Serial.println("I2C ready");
  Serial.print("minRotEncoderAbility:");
  Serial.println(minRotEncoderAbility);
  /*
    compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
    -3076, -3378, -2796
    };
    compass.m_max = (LSM303::vector<int16_t>) {
    +2654, +2189, +2991
    };
  */
#if defined(I2CSlaveMode)
#else
  compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
    compassMin1, compassMin2, compassMin3
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    compassMax1, compassMax2, compassMax3
  };
#endif
  //     compass.read();
  //    refAccX=abs(compass.a.x)*1.1;
  // northOrientation = NorthOrientation();            // added 24/10/2016

}

void loop() {

#if defined(debugLoop)
  if (millis() - timeLoop > 1000)
  {
    debugPrintLoop();                     // for degub only
    timeLoop = millis();
  }
#endif
  CheckEndOfReboot();                      // check for reboot completion
  if (millis() - OutputRobotRequestPinTimer >= minimumDurationBeetwenPolling / 2)
  {
    digitalWrite(RobotOutputRobotRequestPin, LOW);      // reset the request to the subsystem
  }
  // ***  keep in touch with the server
  int getSerial = Serial_have_message();  // check if we have received a message
  if (getSerial > 0)                      // we got a message
  {
    TraitInput(DataInSerial[2]);          // analyze the message
  }
  if (PendingReqSerial != 0x00 )           // check if we have a message to send
  {
#if defined(debugConnection)
    Serial.println("send");
#endif
    DataToSendSerial();                    // send message on the serial link
    timeSendSecSerial = millis();          // reset timer
  }
  if (retryCount >= 5)                    // check retry overflow
  {
    pendingAckSerial = 0x00;               // clear retry flag
    retryCount = 0;                       // clear retry count
  }
  if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 5) {
    ReSendSecuSerial();                    // re-send data that was not already ack by the server
#if defined(debugConnection)
    Serial.println("retry");
#endif
    timeSendSecSerial = millis();         // clear timer
    retryCount = retryCount + 1;          // update rerty count
  }
  if (retryCount >= 5)
  {
    retryCount = 0;
    pendingAckSerial = 0x00;
  }
  if (millis() - timeReceiveSerial >= 30000)     // 30 seconds without receiving data from the server
  {
    bitWrite(diagConnection, diagConnectionIP, 1);       // conection broken
  }

  if (millis() - timeSendInfo >= delayBetweenInfo && actStat != 0x66 && pendingAckSerial == 0x00) // alternatively send status and power to the server
  {
    //   Serial.println(leftMotor.CheckMotor(1, 1), HEX);
    if (sendInfoSwitch % 4 == 1)
    {
      SendStatus();                 // send robot status to the server
    }
    if (sendInfoSwitch % 4 == 2)
    {
      SendPowerValue();             // send power info to the server
    }
    if (sendInfoSwitch % 4 == 3)
    {
      SendEncodersHolesValues();             //
    }
    if (sendInfoSwitch % 4 == 0)
    {
      SendEncoderValues();
    }
    sendInfoSwitch ++;
    timeSendInfo = millis();
  }
  // *** end loop keep in touch with the server

  // *** power check
  if ((millis() - timePowerCheck) >= delayPowerCheck && toDo == 0x00 ) // regularly check power values
  {
    PowerCheck();
  }
  // *** end of loop power check

  // *** refresh LED
  if (millis() - timeAffLed > 1000)
  {
    AffLed();
    timeAffLed = millis();
  }
  // *** end of loop refresh LED

  // *** actions loops
  if (rebootPhase == 0 && appStat != 0xff)         //  wait reboot complete before doing anything
  {

    // *** checking resqueted actions
    if (bitRead(toDo, toDoScan) == 1)          // check if for scan is requested
    {
      ScanPosition();
    }
    if (bitRead(toDo, toDoMove) == 1 && bitRead(waitFlag, toWait) == 0) {      // check if  move is requested
      //  Serial.println("move");
      ComputeTargetLocalization(reqAng, reqMove);
      Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
      appStat = appStat | 0xf0;
    }
    // *** end of checking resqueted actions

    // *** wheels speed control and localization

    if ( millis() - delayCheckPosition > delayBetweenCheckPosition  && bitRead(currentMove, toDoStraight) == true && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    { // Update robot position
      delayCheckPosition = millis();  // reset timer
      if (timeMotorStarted != 0 && millis() - timeMotorStarted > 400 && bitRead(diagMotor, diagMotorPbSynchro) == false) // wait for motors to run enough
      {
        Serial.println("check syncrho");
        //        Serial.println(millis() - timeMotorStarted);
        //        Serial.println(bitRead(diagMotor, diagMotorPbSynchro));
        CheckMoveSynchronisation();
      }
      ComputeNewLocalization(0x00);   // compute dynamic localization
    }
#if defined(debugWheelControlOn)
    if (millis() - delayPrintSpeed > 500 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    {
      PrintSpeed();
    }
#endif
    // *** end of loop wheels control speed and localization

    // *** checking for obstacles
    if (bitRead(waitFlag, toPause) == 1 && (millis() - checkObstacleTimer) > 500 )    // robot in pause status
    {
      CheckNoMoreObstacle();                    // does the obstacle disappeared ?
      checkObstacleTimer = millis();            // reset timer
      if ( bitRead(waitFlag, toEndPause) == 1 ) // no more obstacle
      {
        resumeCount++;
        pauseSince = millis();
        if (resumeCount > 3)                    // "double check" before reume move
        {
          bitWrite(waitFlag, toPause, 0);       // clear wait & pause flag
          bitWrite(waitFlag, toEndPause, 0);
          bitWrite(waitFlag, toWait, 0);
          resumeCount = 0;                      // clear count
          //         ResumeMove();                         // resume move
          ResumeMoveAfterPause();

        }
      }
    }
    if (bitRead(waitFlag, toPause) == 1 && (millis() - pauseSince) > maxPauseDuration)
    {
#if defined(debugObstacleOn)
      Serial.print("end of pause due to timer");
      Serial.println((millis() - pauseSince));
#endif
      bitWrite(currentMove, toDoStraight, false) ;        // clear flag todo straight
      bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo straight
      bitWrite(waitFlag, toPause, 0);       // clear wait & pause flag
      bitWrite(waitFlag, toEndPause, 0);
      bitWrite(waitFlag, toWait, 0);
      resumeCount = 0;                      // clear count
      toDo = 0x00;                                         // clear flag todo
      actStat = moveEnded;                                      // status move completed
      SendEndAction(moveEnded, moveKoDueToObstacle);                       // move not completed due to obstacle
    }
    // *** end of loop checking obstacles

    // *** checking for move completion

    if (bitRead(toDo, toDoStraight) == true && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false)
    {
#if defined(debugMoveOn)
      Serial.println("ready to go straight");
#endif
      if ( bitRead(currentMove, toDoRotation) == true )  // end of first move step (rotation) move straight to be done
      {
#if defined(debugMoveOn)
        Serial.println("end rotation still straight move todo");
        bitWrite(currentMove, toDoRotation, false);
        //       bitWrite(currentMove, toDoStraight, true);
#endif
      }
      MoveForward(pendingStraight) ;
    }

    // *** end of checking move completion

  }
  if (bitRead(toDo, toDoPingFB) == true && millis() - timePingFB > 500)
  {
    SendEndAction(pingFBEnded, 0x00);
    bitWrite(toDo, toDoPingFB, 0);
  }
  if (digitalRead(hornPin) == true && timerHorn < millis())
  {
    Horn(false, 0);
  }
  if (millis() - timeSubsystemPolling > 5000 && rebootPhase == 0)
  {
    timeSubsystemPolling = millis();
    RequestForPolling();
  }
  if (millis() - timeGyroRotation > 500 && bitRead(toDoDetail, toDoGyroRotation) == 1 && leftMotor.CheckMotor(1, 1) == 0 && rightMotor.CheckMotor(1, 1) == 0) //
  {
    timeGyroRotation = millis();
    Serial.println("loop gyro");
    if (gyroRotationRetry >= maxGyroRotationRetry)
    {
      Serial.println("gyro to many retry");
      bitWrite(diagRobot, diagRobotGyroRotation, 1);
      bitWrite(toDoDetail, toDoGyroRotation, 0);
      SendEndAction(moveEnded, 0xfe);
    }
    else
    {
      if (gyroUpToDate == 0x00)
      {
        Serial.println("request heading");
        GyroGetHeadingRegisters();
        gyroUpToDate = 0x01;
      }
      if (gyroUpToDate == 0x02)
      {
        Serial.println("got heading");
        gyroUpToDate = 0x00;
        uint8_t retCode = GyroscopeRotate();
        if (abs(retCode) > 1)
        {
          ComputeAngleAndPositionVSGyro(gyroscopeHeading[gyroscopeHeadingIdx]);
          bitWrite(toDoDetail, toDoGyroRotation, 0);
          bitWrite(diagRobot, diagRobotGyroRotation, 1);
          SendEndAction(moveEnded, retCode);
        }
        if (retCode == -1)
        {
          //          Serial.println(".");
          //         GyroStartInitMonitor;
          //        timeGyroRotation = millis() + 500;
        }
      }
    }
  }
  if (millis() - timeUpdateNO >= delayBetween2NO && gyroUpToDate != 0x01) // regurarly request North orientation if no other request pending
  {
    timeUpdateNO = millis();
    GetNorthOrientation();
  }
}




void Move(int orientation, int lenghtToDo)
{
  //  deltaNORotation = 0;
  //  GyroGetHeadingRegisters();
  NOBeforeRotation = 0;
  NOAfterRotation = 0;
  NOBeforeMoving = 0;
  NOAfterMoving = 0;
  bitWrite(toDo, toDoMove, false);       // position bit toDo move
  if (orientation != 0)
  {
    Rotate(orientation);
  }

  if (lenghtToDo != 0)
  {
    pendingStraight = lenghtToDo;
  }
}
void Rotate( int orientation) {
  //  GyroGetHeadingRegisters();
  // EchoServoAlign();
  // trigCurrent = trigFront;
  // trigBoth = true;         // need to scan front and back during rotation for obstacle detection
  //  echoCurrent = echoFront; // start echo front
  // StartEchoInterrupt(1, 1);
  posRotationCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180);  // save rotation center x position
  posRotationCenterY = posY + shiftEchoVsRotationCenter * sin(alpha * PI / 180);  // save rotation center y position
  NOBeforeRotation = NorthOrientation();
  currentMove = 0x00;
  saveCurrentMove = 0x00;
  bitWrite(currentMove, toDoRotation, true);
  bitWrite(saveCurrentMove, toDoRotation, true);
  bitWrite(toDo, toDoRotation, false);       // position bit toDo move
  deltaPosX = 0;
  deltaPosY = 0;
  unsigned int lentghLeftToDo = 0;
  unsigned int lentghRightToDo = 0;
  if (orientation != 0)
  {
    Serial.print("rotationToDo:");
    Serial.println(orientation);
    // Rotate
    lentghLeftToDo = RotationCalculation(abs(orientation));
    lentghRightToDo = lentghLeftToDo;
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, leftMotorPWM * leftRotatePWMRatio, lentghRightToDo, rightMotorPWM * rightRotatePWMRatio);
    if (orientation > 0)
    {
      bLeftClockwise = true;
      bitWrite(currentMove, toDoClockwise, true);
      bitWrite(saveCurrentMove, toDoClockwise, true);
    }
    else {
      bLeftClockwise = false;
      bitWrite(currentMove, toDoClockwise, false);
      bitWrite(saveCurrentMove, toDoClockwise, false);
    }
    bRightClockwise = bLeftClockwise;

    startMotors();
  }
  Serial.println("rotate ");

}

void MoveForward(int lengthToDo ) {
  //  GyroGetHeadingRegisters();
  delay(10);
  EchoServoAlign(90);
  NOBeforeMoving = NorthOrientation();
  bitWrite(toDo, toDoStraight, false);       // position bit toDo move
  //  currentMove = 0x00;
  bitWrite(currentMove, toDoStraight, true);
  bitWrite(saveCurrentMove, toDoStraight, true);
  unsigned int lentghLeftToDo = 0;
  unsigned int lentghRightToDo = 0;
  deltaPosX = 0;
  deltaPosY = 0;
  trigBoth = false;
  SendStatus();
  // move Toward
  if (lengthToDo != 0 )
  {
    Serial.print("moveToDo:");
    Serial.println(lengthToDo);
    lentghLeftToDo = abs(lengthToDo);
    lentghRightToDo = abs(lengthToDo);
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, leftMotorPWM, lentghRightToDo, rightMotorPWM);
    boolean doEchoFront;
    boolean doEchoBack;

    if (lengthToDo < 0)
    {
      bLeftClockwise = bForward;
      bRightClockwise = !bForward;
      moveForward = false;
      movePingInit = -PingBack();
      echoCurrent = echoBack;
      trigCurrent = trigBack;
      doEchoFront = false;
      doEchoBack = true;
    }
    else
    {
      bLeftClockwise = !bForward;
      bRightClockwise = bForward;
      moveForward = true;
      movePingInit = PingFront();
      echoCurrent = echoFront;
      trigCurrent = trigFront;
      doEchoBack = false;
      doEchoFront = true;
    }
    trigBoth == false;
    unsigned int distF = 0;
    unsigned int distB = 0 ;
    /*
      if (abs(movePingInit) > abs(lengthToDo))
      {
      distF = (abs(movePingInit) - abs(lengthToDo)) * doEchoFront * .9;
      distB = (abs(movePingInit) - abs(lengthToDo)) * doEchoBack * .9;
      }
    */
    distF = (frontLenght + securityLenght) * doEchoFront;
    distB = (backLenght + securityLenght) * doEchoBack;
#if defined(debugObstacleOn)
    Serial.print("Starting obstacle distance:");
    Serial.print(movePingInit);
    Serial.print(" dist echo front:");
    Serial.print(distF);
    Serial.print(" back:");
    Serial.println(distB);
#endif
    if (obstacleDetectionOn)
    {
      StartEchoInterrupt(doEchoFront, distF , doEchoBack, distB );
    }
    startMotors();
  }
}

float  RotationCalculation(int orientation) {
  float distToDo = round(((iRobotWidth * PI / 360) * orientation));
  Serial.print("distRot:");
  Serial.println(distToDo);
  return (distToDo);
}



void PowerCheck()
{
  timePowerCheck = millis();                       // reset timer
  //  Serial.print("power1:");
  unsigned int power1 = analogRead(power1Value); // read power1 analog value and map to fit real voltage
  power1 = analogRead(power1Value);              // read twice to get a better value
  power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power1Mesurt < power1LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 0, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 0, false);                                       // set diag bit power 1 pk
  }
  unsigned int power2 = analogRead(power2Value); // read power1 analog value and map to fit real voltage
  power2 = analogRead(power2Value);               // read twice to get a better value
  power2Mesurt = (power2 * float (990) / 1023);  // map to fit real voltage
#if defined(debugPowerOn)
  Serial.print(power1);
  Serial.print(" ");
  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 9v
  Serial.println("cV 1");
  Serial.print(power2); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print(" ");
  Serial.print(power2Mesurt);
  Serial.println("cV 2");
#endif
  if (power2Mesurt < power2LowLimit)                // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 1, true);                  // set diag bit power 2 ok
    //  myservo.detach();
  }
  else
  {
    bitWrite(diagPower, 1, false);                      // set diag bit power 2 ok
  }

  if (diagPower >= 0x03 && appStat != 0xff)             // global power issue set appStat flag
  {
    //   StopAll();
  }

}

void startMotors()
{
  GyroGetHeadingRegisters();
  digitalWrite(encoderPower, HIGH);
  currentLeftWheelThreshold = iLeftCentiRevolutions;
  currentRightWheelThreshold = iRightCentiRevolutions;
  if (iLeftCentiRevolutions > 0 && iRightCentiRevolutions > 0)
  {
    bitWrite(pendingAction, pendingLeftMotor, true);
    bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
    Serial.print("reqSpeed:");
    Serial.print(iLeftRevSpeed);
    Serial.print(" ");
    Serial.println(iRightRevSpeed);
#endif
    diagMotor = 0x00;
    saveLeftWheelInterrupt = 0;
    saveRightWheelInterrupt = 0;
    digitalWrite(wheelPinInterrupt, LOW);
    attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);
    Wheels.StartWheelControl(true, true, iLeftCentiRevolutions , true, true, iRightCentiRevolutions , false, false, 0, false, false, 0);
    leftMotor.RunMotor(bLeftClockwise,  iLeftRevSpeed);
    rightMotor.RunMotor(bRightClockwise,  iRightRevSpeed);
    timeMotorStarted = millis();
  }
}

void pulseMotors(unsigned int pulseNumber)
{
  GyroGetHeadingRegisters();
  // bitWrite(pendingAction, pendingLeftMotor, true);
  //  bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
  Serial.print("reqSpeed:");
  Serial.print(leftMotorPWM);
  Serial.print(" ");
  Serial.println(rightMotorPWM);
#endif
  diagMotor = 0x00;
  //  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);
  Wheels.StartWheelPulse(pulseNumber);
  leftMotor.RunMotor(bLeftClockwise,  leftMotorPWM);
  rightMotor.RunMotor(bRightClockwise,  rightMotorPWM);
  timeMotorStarted = millis();

}

void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, unsigned int leftMotorPWM, unsigned long iRightDistance, int rightMotorPWM)
{
  // float powerAdustment = float (uRefMotorVoltage) / (3 * map(analogRead(power1Value), 0, 1023, 0, 467)); // speed adjustement to power supply
  float fLeftHoles = (float (iLeftDistance) * leftWheelEncoderHoles / iLeftTractionDistPerRev) ; // Centi-revolutions
  float fRightHoles  = (float (iRightDistance) * rightWheelEncoderHoles / iRightTractionDistPerRev) ; // ms
  iLeftCentiRevolutions = int(round(fLeftHoles));
  if (fLeftHoles - iLeftCentiRevolutions > 0.5)
  {
    iLeftCentiRevolutions++;
  }
  iRightCentiRevolutions = int(round(fRightHoles));
  if (fRightHoles - iRightCentiRevolutions > 0.5)
  {
    iRightCentiRevolutions++;
  }
  iLeftRevSpeed = leftMotorPWM ; // revolutions per minute
  iRightRevSpeed = rightMotorPWM * leftToRightDynamicAdjustRatio ; // revolutions per minute
#if defined(debugMotorsOn)
  Serial.print("compute L:");
  Serial.print(iLeftCentiRevolutions);
  Serial.print(" R:");
  Serial.println(iRightCentiRevolutions);
#endif
}

void CheckMoveSynchronisation()       // check that the 2 wheels are rotating at the same pace
{
  unsigned int currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  float deltaMove = abs(currentLeftHoles - currentRightHoles);
  bitWrite(diagMotor, diagMotorPbSynchro, 0);       // position bit diagMotor
  bitWrite(diagMotor, diagMotorPbLeft, 0);       // position bit diagMotor
  bitWrite(diagMotor, diagMotorPbRight, 0);       // position bit diagMotor
  boolean pbSynchro = false;
#if defined(debugMotorsOn)
  Serial.print("check move synch prevleft:");
  Serial.print(prevCheckLeftHoles);
  Serial.print(" currentleft:");
  Serial.print(currentLeftHoles);
  Serial.print(" prevright:");
  Serial.print(prevCheckRightHoles);
  Serial.print(" currentrightt:");
  Serial.println(currentRightHoles);
#endif
  if (currentLeftHoles == 0 || currentLeftHoles == prevCheckLeftHoles)
  {
    pbSynchro = true;
    bitWrite(diagMotor, diagMotorPbLeft, 1);       // position bit diagMotor
  }
  if (currentRightHoles == 0 || currentRightHoles == prevCheckRightHoles)
  {
    pbSynchro = true;
    bitWrite(diagMotor, diagMotorPbRight, 1);       // position bit diagMotor
  }

  if ( (deltaMove > 3 && deltaMove / float(currentLeftHoles)   > 0.15)  )
  {
    pbSynchro = true;
  }

  if (pbSynchro == true)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    timeMotorStarted = 0;
    delay(100);
    GyroGetHeadingRegisters();
    StopEchoInterrupt(true, true);                    // stop obstacles detection
    Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
    digitalWrite(encoderPower, LOW);
    detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
    bitWrite(diagMotor, diagMotorPbSynchro, 1);       // position bit diagMotor
    pinMode(wheelPinInterrupt, INPUT);
#if defined(debugMotorsOn)
    Serial.print("move synchro pb deltaMove:");
    Serial.print(deltaMove);
    Serial.print(" ");
    Serial.println(deltaMove / float(currentLeftHoles));
#endif

    if (bitRead(currentMove, toDoRotation) == true)
    {
      ComputeNewLocalization(0x01);
    }
    else {
      ComputeNewLocalization(0xff);
    }
    SendEndAction(moveEnded, moveKoDueToSpeedInconsistancy);
#if defined(debugMotorsOn)
    Serial.print("min level left:");
    Serial.print(Wheels.GetMinLevel(leftWheelId));
    Serial.print(" max:");
    Serial.println(Wheels.GetMaxLevel(leftWheelId));
    Serial.print("min level right:");
    Serial.print(Wheels.GetMinLevel(rightWheelId));
    Serial.print(" max:");
    Serial.println(Wheels.GetMaxLevel(rightWheelId));
#endif
    Horn(true, 750);
  }
  prevCheckLeftHoles = currentLeftHoles;
  prevCheckRightHoles = currentRightHoles;
}

void ComputeNewLocalization(uint8_t param)  // compute localization according to the wheels rotation
{
#if defined(debugLoop)
  Serial.print("comp new loc:0x");
  Serial.print(param, HEX);
  debugPrintLoop();
#endif
  unsigned int currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  float deltaAlpha = 0; // modification of robot angles (in degres) since last call of the function
  float deltaLeft = ((float (currentLeftHoles) - saveLeftWheelInterrupt) * PI * iLeftWheelDiameter / leftWheelEncoderHoles); // number of centimeters done by left wheel since last call of the function
  float deltaRight = ((float (currentRightHoles) - saveRightWheelInterrupt) * PI * iRightWheelDiameter / rightWheelEncoderHoles); // number of centimeters done by right wheel since last call of the function
  float alphaRadian = alpha * PI / 180;
  if (bitRead(currentMove, toDoStraight) == true )  // is robot  supposed to go straight ?
  {
    if (bLeftClockwise == true)
    {
      deltaLeft = -deltaLeft;                      // compute move depending on rotation sens
    }

    if (bRightClockwise == false)
    {
      deltaRight = -deltaRight;                    // compute move depending on rotation sens
    }

    if (deltaLeft != 0 && deltaRight != 0)
    {
#if defined(debugWheelControlOn)
      Serial.print("newInt:");
      Serial.print(currentLeftHoles);
      Serial.print(" ");
      Serial.print(saveLeftWheelInterrupt);
      Serial.print(" ");
      Serial.print(currentRightHoles);
      Serial.print(" ");
      Serial.print(saveRightWheelInterrupt);
      Serial.println(" ");
#endif
      saveLeftWheelInterrupt = currentLeftHoles;
      saveRightWheelInterrupt = currentRightHoles;
      float deltaD = (deltaRight - deltaLeft);

      if (deltaD != 0)
      {
        ComputeAngleAndPosition(deltaD, deltaLeft, deltaRight, param);
      }
      else
      {
        deltaPosX = deltaPosX +  deltaLeft * cos(alphaRadian);
        deltaPosY = deltaPosY +  deltaRight * sin(alphaRadian);
      }




      /*
        float deltaD=(deltaRight-deltaLeft)/2;
        deltaAlpha=2*asin(deltaD/(2*iRobotWidth))*180/PI;
        alpha=alpha+deltaAlpha;
        deltaPosX = deltaPosX + deltaC * cos(alpha * PI / 180);
        deltaPosY = deltaPosY + deltaC * sin(alpha * PI / 180);
      */
      /*
        float deltaC = (deltaLeft + deltaRight) / 2; // Move of center of robot (middle of the 2 wheels) in centimeters.
        deltaAlpha = asin((deltaRight - deltaLeft) / (iRobotWidth )); // Compute the modification of the angle since last call of the function
        deltaPosX = deltaPosX + deltaC * cos(alpha * PI / 180);
        deltaPosY = deltaPosY + deltaC * sin(alpha * PI / 180);
        alpha = (deltaAlpha * PI / 180 + alpha);
      */
#if defined(debugLocalizationOn)
      Serial.print(" alpha:");
      Serial.println(alpha);
      Serial.print("deltaPosX:");
      Serial.print(deltaPosX);
      Serial.print(" deltaPosY:");
      Serial.println(deltaPosY);
#endif
    }

  }
  if (param == 0x01 && rotationType != rotationTypeGyro)
  {
    deltaAlpha = ((deltaRight + deltaLeft) * 180 / (PI * iRobotWidth )); //
    if (bitRead(currentMove, toDoClockwise) == false )
    {
      deltaAlpha = -deltaAlpha;
    }
    alpha = int((deltaAlpha + alpha)) % 360;
    posX = posRotationCenterX + shiftEchoVsRotationCenter * cos(alpha * PI / 180);    // to take into account the distance beetwen (X,Y) reference and rotation center
    posY = posRotationCenterY + shiftEchoVsRotationCenter * sin(alpha * PI / 180);
#if defined(debugLocalizationOn)
    Serial.print("new pos X:");
    Serial.print(posX);
    Serial.print(" Y:");
    Serial.print(posY);
    Serial.print(" delta alpha:");
    Serial.print(deltaAlpha);
    Serial.print(" alpha:");
    Serial.println(alpha);
#endif
    //   bitWrite(pendingAction, pendingLeftMotor, false);
    //   bitWrite(pendingAction, pendingRightMotor, false);
  }
  if (param == 0xff )
  {
    //    if (bitRead(currentMove, toDoStraight) == true)
    //    {
    posX = posX + deltaPosX;
    posY = posY + deltaPosY;
    /*
          if (currentRightHoles - currentLeftHoles != 0)
          {
            float posXDyn = posX;
            float posYDyn = posY;
            float alphaDyn = alpha;
            ComputeAngleAndPosition(float (currentRightHoles - currentLeftHoles), currentLeftHoles, currentRightHoles, param);
            posX = (deltaPosX + posXDyn) / 2;
            posY = (deltaPosY + posYDyn) / 2;
            alpha = (alpha + alphaDyn) / 2;
          }
    */
    //   }
#if defined(debugLocalizationOn)
    Serial.print("new pos X:");
    Serial.print(posX);
    Serial.print(" Y:");
    Serial.print(posY);
    Serial.print(" angle:");
    Serial.println(alpha);
#endif
  }
}

void ComputeAngleAndPosition(float deltaD, float deltaLeft, float deltaRight, uint8_t param)
{
  float deltaAlphaRadian = asin(deltaD / (2 * iRobotWidth));
  float rayon =  deltaRight / (2 * sin(deltaAlphaRadian));
  float arcCenter = 2 * (rayon - iRobotWidth / 2) * sin(deltaAlphaRadian);
  float alphaRadian = alpha * PI / 180;
  deltaPosX = deltaPosX + arcCenter * cos(alphaRadian + deltaAlphaRadian);
  deltaPosY = deltaPosY + arcCenter * sin(alphaRadian + deltaAlphaRadian);
  alpha = (deltaAlphaRadian * 2 * 180 / PI + alpha);
#if defined(debugLocalizationOn)
  Serial.print("delta alpha:");
  Serial.print(deltaAlphaRadian * 2 * 180 / PI);
  Serial.print(" ");
#endif
}
void ComputeAngleAndPositionVSGyro(int gyroRot)
{
  //  float deltaAlphaRadian = (gyroRot * PI) / 180;
  posX = posRotationGyroCenterX + shiftEchoVsRotationCenter * cos(gyroRot * PI / 180);    // to take into account the distance beetwen (X,Y) reference and rotation center
  posY = posRotationGyroCenterY + shiftEchoVsRotationCenter * sin(gyroRot * PI / 180);

#if defined(debugLocalizationOn)
  Serial.print(" alpha:");
  Serial.print(gyroRot);
  Serial.print(" ");
#endif
}
void PrintSpeed()
{
  delayPrintSpeed = millis();
  Serial.print("leftRPS:");
  Serial.print(Wheels.GetLastTurnSpeed(0));
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(0));
  Serial.print(" rightRPS:");
  Serial.print(Wheels.GetLastTurnSpeed(1));
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(1));
  Serial.print(" leftHoles:");
  Serial.print(Wheels.GetCurrentHolesCount(0));
  Serial.print(" rightHoles:");
  Serial.println(Wheels.GetCurrentHolesCount(1));
}




void AffLed()
{
  if (diagPower == 0x00)
  {
    digitalWrite(redLed, 1);
    redLedOn = true;
  }
  else
  {
    if (diagPower == 0b00000111)
    {
      digitalWrite(redLed, 0);
      redLedOn = false;
    }
    else
    {
      redLedOn = !redLedOn;
      digitalWrite(redLed, redLedOn);
    }
  }

  if (diagConnection == 0x00)
  {
    digitalWrite(blueLed, 1);
    blueLedOn = true;
  }
  else
  {
    blueLedOn = !blueLedOn;
    digitalWrite(blueLed, blueLedOn);
  }

  if (diagRobot == 0x00  )
  {
    digitalWrite(greenLed, 1);
    greenLedOn = true;
  }
  else
  {
    greenLedOn = !greenLedOn;
    digitalWrite(greenLed, greenLedOn);
  }
  if (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true)
  {
    if (diagMotor == 0x00)
    {
      //    Serial.println(diagMotor, HEX);
      digitalWrite(yellowLed, 1);
      yellowLedOn = true;
    }
    else
    {
      yellowLedOn = !yellowLedOn;
      digitalWrite(yellowLed, yellowLedOn);
    }
  }
  else
  {

    if (diagMotor == 0x00)
    {

      digitalWrite(yellowLed, 0);
      yellowLedOn = false;
    }
    else
    {
      yellowLedOn = !yellowLedOn;
      digitalWrite(yellowLed, yellowLedOn);
    }
    if (appStat == 0xff)
    {
      digitalWrite(greenLed, 1);
      greenLedOn = false;
    }
  }
}


void StopAll()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  timeMotorStarted = 0;
  myservo.detach();
  StopEchoInterrupt(true, true);
  Wheels.StopWheelControl(true, true, 0, 0);
  GyroStopMonitor();
  digitalWrite(encoderPower, LOW);
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  pinMode(wheelPinInterrupt, INPUT);
  appStat = 0xff;       // update robot status
  actStat = 0x00;       // clear action status
  toDo = 0x00;           // cleat toDo byte
  toDoDetail = 0x00;           // cleat toDo byte
  currentMove = 0x00;    // clear current move
  saveCurrentMove = 0x00; // clear saveCurrent move
  pendingAckSerial = 0x00; // clear pending acknowledgement
  timerHorn = 0;          // reset horn timer
  SendStatus();
}

void StartEchoInterrupt(boolean frontOn, unsigned int frontL, boolean backOn, unsigned int backL)
{

  frontL = max(frontLenght + securityLenght, frontL);
  backL = max(backLenght + securityLenght, backL);
  obstacleDetectionCount = 0;
#if defined(debugObstacleOn)
  Serial.print("start echo interrupt ");
  if (frontOn == true)
  {
    Serial.print(" front:");
    Serial.print(frontL);
  }
  if (backOn == true)
  {
    Serial.print( " back:");
    Serial.print(backL);
  }
  Serial.println();
  scanNumber = 0;
#endif
  //  digitalWrite(echoPinInterrupt, HIGH);
  attachInterrupt(digitalPinToInterrupt(echoPinInterrupt), obstacleInterrupt, RISING);
  echo.StartDetection(frontOn, backOn, echo3, echo4, echoCycleDuration);
  echo.SetAlertOn(frontOn, frontL, backOn, backL, echo3Alert, 0, echo4Alert, 0);
}

void StopEchoInterrupt(boolean frontOff, boolean backOff)
{
#if defined(debugObstacleOn)
  Serial.println("stop echo interrupt");
#endif
  echo.StopDetection(!frontOff, !backOff, echo3, echo4);
  detachInterrupt(digitalPinToInterrupt(echoPinInterrupt));
  //  pinMode(echoPinInterrupt, INPUT_PULLUP);
  echoCurrent = 0;
  trigCurrent = 0;
  scanNumber = 0;

}

void CheckNoMoreObstacle()              // check if obstacle still close to the robot
{
  unsigned int dist = 0;
  unsigned int lastMicro;
  int deltaSecurity = 0;
  boolean switchPause = bitRead(waitFlag, toPause);   // detect if we are already in pause status
  dist = echo.GetDistance(echo.GetAlertEchoNumber());
  if (dist != 0)
  {
    if ( dist < echo.GetEchoThreshold(echo.GetAlertEchoNumber()))                 // compare echo with security distance
    {
      bitWrite(waitFlag, toPause, true);       // position bit pause on
      bitWrite(diagRobot, diagRobotObstacle, 1);       // position bit diagRobot
      timeBetweenOnOffObstacle = millis();     // use to compute delay between obstacle detection sitch on off
    }
    else
    {
      if (millis() - timeBetweenOnOffObstacle > delayBeforeRestartAfterAbstacle)
      {
        bitWrite(waitFlag, toEndPause, true);     // position bit pause to end
        bitWrite(diagRobot, diagRobotObstacle, 0);       // position bit diagRobot
        //       digitalWrite(echoPinInterrupt, LOW);      // to reactivate echo interrupt
        Horn(true, 250);
      }
    }
  }

  checkObstacleTimer = millis();  // reset timer

#if defined(debugObstacleOn)
  if (bitRead(waitFlag, toPause) == true && switchPause == false)
  {
    Serial.println("pause due to obstacle");
  }
  Serial.print(" echo number:");
  Serial.print(echo.GetAlertEchoNumber());
  Serial.print(" dist:");
  Serial.print(dist);
  Serial.print(" threshold:");
  Serial.println(echo.GetEchoThreshold(echo.GetAlertEchoNumber()));
#endif

}

void PauseMove()                  //
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  timeMotorStarted = 0;
  pauseDirectionBackward = bitRead(toDo, toDoBackward);
  //  unsigned int currentLeftWheelThreshold = Wheels.GetWheelThreshold(leftWheelId);
  //  unsigned int currentRightWheelThreshold = Wheels.GetWheelThreshold(rightWheelId);
#if debugObstacleOn
  {
    Serial.print("stop obstacle ");
    Serial.print("threshold left:");
    Serial.print(currentLeftWheelThreshold);
    Serial.print(" right:");
    Serial.println(currentRightWheelThreshold);

  }
#endif
  delay(100);
  GyroGetHeadingRegisters();
  pauseLeftWheelInterrupt = currentLeftWheelThreshold - Wheels.GetCurrentHolesCount(leftWheelId);
  pauseRightWheelInterrupt = currentRightWheelThreshold - Wheels.GetCurrentHolesCount(rightWheelId);
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
#if debugObstacleOn
  {
    Serial.print("pause left:");
    Serial.print(pauseLeftWheelInterrupt);
    Serial.print(" right:");
    Serial.println(pauseRightWheelInterrupt);
    Serial.println("stop obstacle");
  }
#endif
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  digitalWrite(encoderPower, LOW);
  pinMode(wheelPinInterrupt, INPUT);
  //  delayToStopWheel = millis();
  if (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true)
  {
    Horn(true, 500);
  }
  bitWrite(pendingAction, pendingLeftMotor, false);
  bitWrite(pendingAction, pendingRightMotor, false);

  if (bitRead(currentMove, toDoRotation) == true)
  {
    ComputeNewLocalization(0x01);
  }
  else {
    ComputeNewLocalization(0xff);
  }
  deltaPosX = 0;
  deltaPosY = 0;
  SendStatus();
}

void ResumeMove()  // no more obstacle resume moving
{
  bitWrite(diagRobot, diagRobotPause, 0);       // position bit diagRobot
  Serial.println("resume move");
  float deltaX = targetX - posX;     // compute move to do to reach target
  //  Serial.println(deltaX);
  float deltaY = targetY - posY;     // compute move to do to reach target
  //  Serial.println(deltaY);
  float deltaAlpha = atan(deltaY / deltaX) * 180 / PI; // compute move to do to reach target
  int rotation =  deltaAlpha - alpha;
  int lenToDo = 0;
  if (moveForward == true)
  {
    if (deltaX < 0)
    {
      rotation = 180 + rotation;
      if (rotation > 180)
      {
        rotation = rotation - 360;   // to optimize move
      }
    }
    lenToDo = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  }
  else                               // was going straight back
  {
    lenToDo = -sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    rotation = 0;
  }
  // if (targetX < posX)
  // {
  //    lenToDo = -lenToDo;
  //  }
#if defined(debugLocalizationOn)
  Serial.print("delta X:");
  Serial.print(deltaX);
  Serial.print(" Y:");
  Serial.println(deltaY);
  Serial.print("rotation:");
  Serial.print(rotation);
  Serial.print(" dist:");
  Serial.println(lenToDo);
#endif
  if (rotation != 0)
  {
    bitWrite(toDo, toDoRotation, true);
  }
  if (lenToDo > 0)
  {
    bitWrite(toDo, toDoStraight, true);
    bitWrite(toDo, toDoBackward, false);
    //       bitWrite(currentMove, toDoClockwise, true);
    //        bitWrite(saveCurrentMove, toDoClockwise, true);
  }
  if (lenToDo < 0)
  {
    bitWrite(toDo, toDoStraight, true);
    bitWrite(toDo, toDoBackward, true);
    //       bitWrite(currentMove, toDoClockwise, false);
    //       bitWrite(saveCurrentMove, toDoClockwise, false);
  }
  if (abs(lenToDo) >= minDistToBeDone || abs(rotation) >= minRotToBeDone)
  {
    pendingStraight = lenToDo;
    Move(rotation, lenToDo);
  }
  else
  {
    actStat = moveEnded;                                      // status move completed
    toDo = 0x00;                                         // clear flag todo
    SendEndAction(moveEnded, moveUnderLimitation);                       //
  }
  SendStatus();
}

void ResumeMoveAfterPause()  // no more obstacle resume moving
{
  bitWrite(diagRobot, diagRobotPause, 0);       // position bit diagRobot
  float lenToDo = (((pauseLeftWheelInterrupt * iLeftWheelDiameter) / leftWheelEncoderHoles + (pauseRightWheelInterrupt * iRightWheelDiameter) / rightWheelEncoderHoles ) * PI / 2);
  bitWrite(toDo, toDoStraight, true);
  if (pauseDirectionBackward == true)
  {
    bitWrite(toDo, toDoBackward, true);
    lenToDo = -lenToDo;
  }
  else
  {
    bitWrite(toDo, toDoBackward, false);
  }
#if debugObstacleOn
  Serial.print("resume move after pause:");
  Serial.println(lenToDo);
#endif
  if (abs(lenToDo) >= minDistToBeDone )
  {
    pendingStraight = lenToDo;
    Move(0, lenToDo);
  }
  else
  {
    actStat = moveEnded;                                      // status move completed
    toDo = 0x00;                                         // clear flag todo
    SendEndAction(moveEnded, moveUnderLimitation);                       //
  }
  SendStatus();
}
void ComputeTargetLocalization(int rotation, int lenghtToDo)
{
  targetAlpha = alpha + rotation;           // target orientation = current orientation + requested rotation
  targetX = posX + cos(targetAlpha * PI / 180) * lenghtToDo;
  targetY = posY + sin(targetAlpha * PI / 180) * lenghtToDo;
#if defined(debugLocalizationOn)
  Serial.print("target orient:");
  Serial.print(targetAlpha);
  Serial.print(" x:");
  Serial.print(targetX);
  Serial.print(" y:");
  Serial.println(targetY);
#endif
}

void CheckBeforeStart()
{
  int newDist = 0;
  Serial.print("check before start");
  if (movePingInit < 0)
  {
    newDist = -PingBack();
    if (abs(newDist) < backLenght + securityLenght)
    {
      bitWrite(waitFlag, toWait, true);       // position bit pause on
      //     PauseMove();
    }
  }
  else
  {
    newDist = PingFront();
    if (abs(newDist) < frontLenght + securityLenght)
    {
      bitWrite(waitFlag, toWait, true);       // position bit pause on
      //     PauseMove();
    }
  }
}

void CheckEndOfReboot()  //
{
  if (rebootPhase == 3 && millis() > rebootDuration) // end of arduino reboot
  {
    Serial.println("reboot phase:3");
    EchoServoAlign(90);                               // adjust servo motor to center position
    delay(1000);
    northOrientation = NorthOrientation();            // added 24/10/2016
    PingFrontBack();
    rebootPhase = 2;
    Serial.println("reboot phase:2");
  }
  if (rebootPhase == 2 && digitalRead(RobotInputReadyPin) == 1 && millis() > rebootDuration + 5000) // end of arduino reboot
  {
    northOrientation = NorthOrientation();            // added 24/10/2016
    PingFrontBack();
    rebootPhase = 1;
    //    CalibrateGyro();
    Serial.println("reboot phase:1");
  }
  if (rebootPhase == 1 && gyroCalibrationOk && millis() > rebootDuration + 7000) // end of arduino reboot
  {
    diagRobot = 0x00;
    waitFlag = 0x00;
    appStat = 0x00;
    bitWrite(diagConnection, diagConnectionI2C, 0);
    PingFrontBack();
    Horn(true, 250);
    rebootPhase = 0;
    Serial.println("reboot completed");
  }
}

void obstacleInterrupt()        // Obstacles detection system set a softare interrupt due to threshold reaching
{
  obstacleDetectionCount++;
  volatile uint8_t echoID = echo.GetAlertEchoNumber();
  volatile unsigned int obstacleDistance = echo.GetDistance(echoID);
#if defined(debugObstacleOn)
  Serial.print("obstacle a:");
  Serial.println(obstacleDistance);
#endif
  if (obstacleDistance != 0 && obstacleDetectionCount >= 2)
  {
    bitWrite(waitFlag, toPause, true);     // position bit pause to end
    bitWrite(diagRobot, diagRobotObstacle, 1);       // position bit diagRobot
    //  digitalWrite(echoPinInterrupt, LOW);
    pauseSince = millis();
    PauseMove();
  }
}

void WheelInterrupt()   // wheel controler set a software interruption due to threshold reaching
{
  uint8_t wheelId = Wheels.GetLastWheelInterruptId();  // get which wheel Id reached the threshold
  if (wheelId != 5)
  {
    Wheels.ClearThreshold(wheelId);                      // clear the threshold flag to avoid any more interruption
  }
  WheelThresholdReached(wheelId);                      // call the threshold analyse
}

void WheelThresholdReached( uint8_t wheelId)
{
  if (wheelId != 5)
  {
    if (wheelId == leftWheelId)
    {
      leftMotor.StopMotor();                             // stop firstly the motor that reached the threshold
      rightMotor.StopMotor();                            // stop the other motor to avoid to turn round
    }
    else
    {
      rightMotor.StopMotor();                         // stop firstly the motor that reached the threshold
      leftMotor.StopMotor();                          // stop the other motor to avoid to turn round
    }

    timeMotorStarted = 0;
    StopEchoInterrupt(true, true);                    // stop obstacles detection
    delay(200);                                      // wait a little for robot intertia
    gyroUpToDate = 0x00;
    GyroGetHeadingRegisters();
    Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
    detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
    pinMode(wheelPinInterrupt, INPUT);
    digitalWrite(encoderPower, LOW);
    leftWheeelCumulative = leftWheeelCumulative + Wheels.GetCurrentHolesCount(leftWheelId);
    rightWheeelCumulative = rightWheeelCumulative + Wheels.GetCurrentHolesCount(rightWheelId);
    unsigned int leftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
    unsigned int rightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
    delayAfterStopMotors = millis();
    //  Serial.println(leftWheeelCumulative - rightWheeelCumulative);
    /*
      if (leftHoles != rightHoles)
      {
      if ((leftWheeelCumulative - rightWheeelCumulative > 0) && (leftHoles > rightHoles) )
      {
        leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio + 0.1;
      }
      if ((leftWheeelCumulative - rightWheeelCumulative < 0) && (leftHoles < rightHoles) )
      {
        leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio - 0.1;
      }
      }
    */

#if defined(debugMoveOn)
    Serial.print("RPM:");
    Serial.print(wheelId);
    Serial.print(" ");
    Serial.print(Wheels.GetLastTurnSpeed(wheelId) * 60);
    Serial.print(" ");
    Serial.println(Wheels.Get2LastTurnSpeed(wheelId) * 60);
    Serial.print("Holesleft:");
    Serial.print(leftHoles);
    Serial.print(" right:");
    Serial.println(rightHoles);
#endif
    /*
        if ( bitRead(currentMove, toDoStraight) == true)      // robot was moving straight
        {
          ComputeNewLocalization(0xff);                       // compute new  robot position
          bitWrite(currentMove, toDoStraight, false) ;        // clear flag todo straight
        }
    */
    bitWrite(pendingAction, pendingLeftMotor, false);     // clear the flag pending action motor
    bitWrite(pendingAction, pendingRightMotor, false);    // clear the flag pending action motor
    if ( bitRead(currentMove, toDoRotation) == true)      // robot was turning around
    {
      ComputeNewLocalization(0x01);                       // compute new  robot position
      bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation
      delay(100);
      NOAfterRotation = NorthOrientation();
      if (bitRead(toDo, toDoStraight) == false && bitRead(toDoDetail, toDoGyroRotation) == 0)
      {
        SendEndAction(moveEnded, 0x00);
      }
    }
    else
    {
      ComputeNewLocalization(0xff);                       // compute new  robot position
      bitWrite(currentMove, toDoStraight, false) ;        // clear flag todo straight
      actStat = moveEnded;                                      // status move completed
      toDo = 0x00;                                         // clear flag todo
      uint8_t retCode = 0x00;
      bitWrite(diagMotor, diagMotorPbEncoder, 0);
      bitWrite(diagMotor, diagMotorPbLeft, 0);
      bitWrite(diagMotor, diagMotorPbRight, 0);
      delay(100);
      NOAfterMoving = NorthOrientation();
      if (Wheels.GetMinLevel(leftWheelId) > leftIncoderLowValue * 1.3)
      {
        retCode = moveRetcodeEncoderLeftLowLevel;
        bitWrite(diagMotor, diagMotorPbLeft, 1);
      }
      if (Wheels.GetMinLevel(rightWheelId) > rightIncoderLowValue * 1.3)
      {
        retCode = moveRetcodeEncoderRightLowLevel;
        bitWrite(diagMotor, diagMotorPbRight, 1);
      }
      if (Wheels.GetMaxLevel(leftWheelId) < leftIncoderHighValue * 0.7)
      {
        retCode = moveRetcodeEncoderLeftHighLevel;
        bitWrite(diagMotor, diagMotorPbLeft, 1);
      }
      if (Wheels.GetMaxLevel(rightWheelId) < rightIncoderHighValue * 0.7)
      {
        retCode = moveRetcodeEncoderRightHighLevel;
        bitWrite(diagMotor, diagMotorPbRight, 1);
      }
      if (retCode != 0x00)
      {
        bitWrite(diagMotor, diagMotorPbEncoder, 1);
#if defined(debugMoveOn)
        Serial.print("pb level encoder minLeft;");
        Serial.print(Wheels.GetMinLevel(leftWheelId));
        Serial.print(" min right:");
        Serial.print(Wheels.GetMinLevel(rightWheelId));
        Serial.print( "max left:");
        Serial.print(Wheels.GetMaxLevel(leftWheelId));
        Serial.print( "max right:");
        Serial.println(Wheels.GetMaxLevel(rightWheelId));
#endif
      }
      SendEndAction(moveEnded, retCode);
    }
    /*
        if ( bitRead(waitFlag, toWait) == 0 && actStat == moving)           // no more move to do
        {
          actStat = moveEnded;                                      // status move completed
          toDo = 0x00;                                         // clear flag todo
          SendEndAction(moveEnded);
        }
    */
  }
  else                      // wheel mode pulse
  {
    leftMotor.StopMotor();                             // stop firstly the motor that reached the threshold
    rightMotor.StopMotor();                            // stop the other motor to avoid to turn round
    timeMotorStarted = 0;
    GyroGetHeadingRegisters();
  }
}

void CalibrateWheels()           // calibrate encoder levels and leftToRightDynamicAdjustRatio according to mesurments
{
  saveLeftWheelInterrupt = 0;
  saveRightWheelInterrupt = 0;
  digitalWrite(encoderPower, HIGH);
  //  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);
  Wheels.StartWheelControl(true, false, 0 , true, false, 0 , false, false, 0, false, false, 0);
  leftMotor.RunMotor(0,  leftMotorPWM );
  rightMotor.RunMotor(1, rightMotorPWM * leftToRightDynamicAdjustRatio);
  delay(4000);                                    // run motors 4 seconds
  rightMotor.StopMotor();                         // stop firstly the motor that reached the threshold
  leftMotor.StopMotor();                          // stop the other motor to avoid to turn round
  StopEchoInterrupt(true, true);                    // stop obstacles detection
  delay(200);                                       // wait a little for robot intertia
  GyroGetHeadingRegisters();
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  digitalWrite(encoderPower, LOW);
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  pinMode(wheelPinInterrupt, INPUT);
  leftWheeelCumulative = leftWheeelCumulative + Wheels.GetCurrentHolesCount(leftWheelId);
  rightWheeelCumulative = rightWheeelCumulative + Wheels.GetCurrentHolesCount(rightWheelId);
  unsigned int minLeftLevel = Wheels.GetMinLevel(leftWheelId);
  unsigned int maxLeftLevel = Wheels.GetMaxLevel(leftWheelId);
  unsigned int minRightLevel = Wheels.GetMinLevel(rightWheelId);
  unsigned int maxRightLevel = Wheels.GetMaxLevel(rightWheelId);
  unsigned int leftAvgLevel = (maxLeftLevel + minLeftLevel) / 2;
  unsigned int leftDiffLevel = (maxLeftLevel - minLeftLevel) / 4;
  unsigned int leftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int rightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  leftIncoderLowValue = leftAvgLevel - leftDiffLevel;
  leftIncoderHighValue = leftAvgLevel + leftDiffLevel;
  unsigned int rightAvgLevel = (maxRightLevel + minRightLevel) / 2;
  unsigned int rightDiffLevel = (maxRightLevel - minRightLevel) / 4;
  rightIncoderLowValue = rightAvgLevel - rightDiffLevel;
  rightIncoderHighValue = rightAvgLevel + rightDiffLevel;
  Serial.println(leftWheeelCumulative - rightWheeelCumulative);
  if (leftHoles != rightHoles)
  {
    Serial.print ("diff ");
    if ((leftWheeelCumulative - rightWheeelCumulative > 0) && (leftHoles > rightHoles) )
    {
      Serial.println("+");
      leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio + 0.1;
    }
    if (( rightWheeelCumulative - leftWheeelCumulative > 0) && ( rightHoles > leftHoles) )
    {
      Serial.println("-");
      leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio - 0.1;
    }
  }
#if defined(debugWheelControlOn)
  Serial.print("min level left:");
  Serial.print(minLeftLevel);
  Serial.print(" max:");
  Serial.println(maxLeftLevel);
  Serial.print("min level right");
  Serial.print(minRightLevel);
  Serial.print(" max:");
  Serial.println(maxRightLevel);
  Serial.print("Holes left:");
  Serial.print(leftHoles);
  Serial.print(" right:");
  Serial.println(rightHoles);
  Serial.print("speed left  ");
  Serial.print(Wheels.GetLastTurnSpeed(leftWheelId) * 60);
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(leftWheelId) * 60);
  Serial.print(" right  ");
  Serial.print(Wheels.GetLastTurnSpeed(rightWheelId) * 60);
  Serial.print(" ");
  Serial.println(Wheels.Get2LastTurnSpeed(rightWheelId) * 60);
  Serial.print("Cumulative Holes left:");
  Serial.print(leftWheeelCumulative);
  Serial.print(" right:");
  Serial.println(rightWheeelCumulative);
  Serial.print("dynamic adjust:");
  Serial.println(leftToRightDynamicAdjustRatio);
  Serial.print("encoder left low:");
  Serial.print(leftIncoderLowValue);
  Serial.print(" high:");
  Serial.println(leftIncoderHighValue);
  Serial.print("encoder right low:");
  Serial.print(rightIncoderLowValue);
  Serial.print(" high:");
  Serial.println(rightIncoderHighValue);
#endif
  SendEncoderMotorValue();
}

void Horn(boolean hornOn, unsigned int hornDuration)  //
{
  if (hornOn == true)
  {
    timerHorn = millis() + hornDuration;
    digitalWrite(hornPin, HIGH);
  }
  else
  {
    digitalWrite(hornPin, LOW);
    timerHorn = 0;
  }
}


int NorthOrientation()
{
#if defined(I2CSlaveMode)
  return 0;
#else
  compass.read();
  float northOrientation = compass.heading();
#if defined(debugMagnetoOn)
  Serial.print(" magneto orientation: ");
  Serial.println(northOrientation);
#endif

  saveNorthOrientation = int(northOrientation);
  return saveNorthOrientation;
#endif
}

#define minAlign 7
#define maxAlign 357
void northAlign(unsigned int northAlignShift)
{ // aligning anti-clockwise positive    - robot rotation clockwise positive
  uint8_t retry = 0;
  boolean aligned = false;
  actStat = aligning; // align required
  while (aligned == false && retry <= 50)
  {
    compass.read();
    saveNorthOrientation = int(compass.heading());
    unsigned int  alignTarget = (saveNorthOrientation + 360 - northAlignShift) % 360;
    if ((alignTarget <= 180 && alignTarget > minAlign) || (alignTarget >= 180 && alignTarget < maxAlign))
    {
#if defined(debugLocalizationOn)
      Serial.print(" align to: ");
      Serial.println(alignTarget);
#endif
      //      toDo = 0x00;
      bitWrite(toDo, toDoAlign, 1);       // position bit toDo
      appStat = appStat & 0x1f;
      if (alignTarget <= 180)
      {
        if (alignTarget > 15)
        {
          Rotate(alignTarget);
        }
        else
        {
          bLeftClockwise = true;
          bRightClockwise = bLeftClockwise;
          //        bitWrite(currentMove, toDoClockwise, true);
          //        bitWrite(saveCurrentMove, toDoClockwise, true);
          pulseMotors(4);
          delay(200);     // to let time for inertia
        }
      }
      else
      {
        if (alignTarget < 345)
        {
          Rotate(alignTarget - 360);
        }
        else
        {
          bLeftClockwise = false;
          bRightClockwise = bLeftClockwise;
          //        bitWrite(currentMove, toDoClockwise, false);
          //        bitWrite(saveCurrentMove, toDoClockwise, false);
          pulseMotors(4);
        }
      }
      delay(1000);
      retry++;
    }
    else
    {
      delay(1000);
      compass.read();  // double check
      saveNorthOrientation = int(compass.heading());
      unsigned int  alignTarget = (saveNorthOrientation + 360 - northAlignShift) % 360;
      if ((alignTarget <= 180 && alignTarget > minAlign) || (alignTarget >= 180 && alignTarget < maxAlign))
      {
        retry++;
      }
      else
      {
        actStat = alignEnded; // align required
        aligned = true;
        if (targetAfterNORotation != 0)         // case rotation based on NO
        {
          alpha = targetAfterNORotation;        // set orientation to the expectation
        }
        if (bitRead(toDo, toDoAlign) == true)
        {
          bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
        }
        SendEndAction(alignEnded, 0x00);
      }
    }
  }
}
uint8_t GyroscopeRotate()
{
  gyroUpToDate = 0x00;
  if (bitRead(monitSubsystemStatus, monitGyroStatusBit) == 1)
  {
    //  GyroGetHeadingRegisters();
    //   timeGyroRotation = millis();
    gyroRotationRetry++;
#if defined(gyroDebugOn)
    Serial.println(gyroRotationRetry);
#endif
    boolean aligned = false;
    actStat = gyroRotating; //
    // gyroTargetRotation=rotation;
    // delay(100);

    if (gyroInitRotationSens != 0x00 && abs(gyroTargetRotation) >= minRotEncoderAbility)     // first step
    {
      gyroInitRotationSens = 0x00;
      prevGyroRotation = gyroTargetRotation;
      OptimizeGyroRotation(gyroTargetRotation);
      if (gyroTargetRotation < 0)
      {
        gyroTargetRotation = 360 + gyroTargetRotation;
      }
#if defined(gyroDebugOn)
      Serial.print("first gyro rotation step:");
      Serial.println(gyroTargetRotation);
#endif
      return (1);
    }
    /*
        if (abs(gyroH) >= 180)
        {
          gyroH = (360 - gyroH);
        }
        else
        {
          gyroH = -gyroH;
        }
    */
    int gyroH = gyroscopeHeading[gyroscopeHeadingIdx];
    int rotation = 0;
    //    gyroH = (360 - gyroH)%360;        // change to anti clockwiseint totation=0;
#if defined(gyroDebugOn)
    Serial.print("gyro relativeheading:");
    Serial.println(gyroH);
#endif
    if ((gyroTargetRotation > 0 && gyroTargetRotation < 90) && (gyroH > 270 && gyroH < 360))
    {
#if defined(gyroDebugOn)
      Serial.print("case 1 over 0:");
      Serial.print(gyroTargetRotation);
      Serial.print(" ");
      Serial.println(gyroH);
#endif
      if (prevGyroRotation >= 0)
      {
        rotation = gyroTargetRotation + (360 - gyroH);
      }
      else                    // more than 360° turn has been done
      {
        return -2;
      }
    }
    else if ((gyroTargetRotation > 270 && gyroTargetRotation < 360) && (gyroH > 0 && gyroH < 90))
    {
#if defined(gyroDebugOn)
      Serial.print("case 1 over 0:");
      Serial.print(gyroTargetRotation);
      Serial.print(" ");
      Serial.println(gyroH);
#endif
      if (prevGyroRotation <= 0 )
      {
        rotation = -(360 - gyroTargetRotation) + gyroH;
      }
      else                    // more than 360° turn has been done
      {
        return -2;
      }

    }
    else
    {
      rotation = (gyroTargetRotation - gyroH) ;
#if defined(gyroDebugOn)
      Serial.print("new rotation:");
      Serial.println(rotation);
#endif
    }
    prevGyroRotation = rotation;
#if defined(gyroDebugOn)
    Serial.print("gyro:");
    Serial.println(rotation);
#endif
    if (abs(rotation) < minRotationTarget)
    {
#if defined(gyroDebugOn)
      Serial.print("end gyro:");
      Serial.println(rotation);
#endif
      aligned = true;
      bitWrite(toDoDetail, toDoGyroRotation, 0);       // clear bit toDo
      alpha = alpha + gyroH;
      ComputeAngleAndPositionVSGyro(gyroH);
      SendEndAction(moveEnded, 0x00);
      return 0;
    }
    else
    {
      if (abs(rotation) < minRotEncoderAbility)
      {
#if defined(gyroDebugOn)
        Serial.print(" gyro pulse: ");
#endif
        if (rotation >= 0)
        {
          bLeftClockwise = true;
        }
        else
        {
          bLeftClockwise = false;
        }
        bRightClockwise = bLeftClockwise;
        //        bitWrite(currentMove, toDoClockwise, false);
        //        bitWrite(saveCurrentMove, toDoClockwise, false);
        pulseMotors(4);
      }
      else
      {
#if defined(gyroDebugOn)
        Serial.print(" gyro rotate: ");
        Serial.println(rotation);
#endif
        //      toDo = 0x00;


        appStat = appStat & 0x1f;
        OptimizeGyroRotation(rotation);
        ///     if (abs(rotation) > 15 && gyroRotationRetry <= 3)
        //     {

      }
      return (1);
    }
  }
  else
  {
    ResetGyroscopeHeadings();
    GyroStartInitMonitor();
    Serial.println("start gyro monitor");

    return -1;
  }
}


void debugPrintLoop()
{
  Serial.print(" currentMove:0x");
  Serial.print(currentMove, HEX);
  Serial.print(" todo:0x");
  Serial.print(toDo, HEX);
  Serial.print(" pending Straight:");
  Serial.print(pendingStraight);
  Serial.print(" waitFlag:0x");
  Serial.print(waitFlag, HEX);
  Serial.print(" diagRobot:0x");
  Serial.print(diagRobot, HEX);
  Serial.print(" pendingAction:0x");
  Serial.println(pendingAction, HEX);
}


void resetStatus(uint8_t code)
{
  if (bitRead(code, resetMotor) == true)
  {
    diagMotor = 0x00;
  }
  if (bitRead(code, resetPause) == true)
  {
    bitWrite(diagRobot, diagRobotPause, 0);
    waitFlag = 0x00;
  }
  if (bitRead(code, resetObstacle) == true)
  {
    bitWrite(diagRobot, diagRobotObstacle, 0);
  }
  pendingAction = 0x00;
  toDo = 0x00;
  currentMove = 0x00;
  timerHorn = 0;
}

void OptimizeGyroRotation (int rotation)
{
  if (abs(rotation) > 3 * maxInertialRotation)
  {
    if (rotation >= 0)
    {
      Rotate(rotation - maxInertialRotation);
#if defined(gyroDebugOn)
      Serial.println( rotation - maxInertialRotation);
#endif
    }
    else
    {
      Rotate(rotation + maxInertialRotation);
#if defined(gyroDebugOn)
      Serial.println( rotation + maxInertialRotation);
#endif
    }
  }
  else
  {
    if (rotation >= 0)
    {
      Rotate(max(round(rotation * 0.8), minRotEncoderAbility));
#if defined(gyroDebugOn)
      Serial.println( max(round(rotation * 0.8), minRotEncoderAbility));
#endif
    }
    else
    {
      Rotate(min(round(rotation * 0.8), -minRotEncoderAbility));
#if defined(gyroDebugOn)
      Serial.println( min(round(rotation * 0.8), -minRotEncoderAbility));
#endif
    }
  }
}


/*
  void readParameters(uint8_t number, uint8_t list[maxReadParamtersList])
  {
  number = min(number, maxReadParamtersList);
  //     Wire.beginTransmission(4);
  Wire.write("2", 1);             // sends one byte
  }
*/




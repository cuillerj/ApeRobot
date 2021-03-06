/*
   v2 a nano added as sensor subsytem for gyroscope

   release notes
   v2.2 new serial gateway
   v2.3 power monitoring improved
        sonar test before rotation modified
        check wheels running during rotation
   v3.0 move acrass narrow path under development
   v3.1 reboot modified to enventualy move a little to calibrate compass on demand of the subsystem
   v3.2 NOUpdate modification
   v3.3 suppress check wheel speed during rotation
   v3.4 one LED added that lights when action is pending
   v3.5 extend scan control before north align
   v3.6 wheel moving control modified for rotation
   v3.7 ajout trace north orientation for tunning
   v3.8 add sleep mode
   v3.9 modified updateNo and sendend to delay status report
   v3.10 debug obstacle move back detection
   v3.11 debug send scan360 info
   v3.12 modification gestion timer scan
   v4.0 newping call rewritten
   v4.1 version management evolution (version subversion) remotly accessible
   v4.2 optimize rotation for north alignement and debug echoMinFB
   v4.3 last action retcode replace locprob in status frame
   v4.4 modif input rotate gyro
   v4.5 new WheelControl with delayMaxBetweenHoles
   v5.0 motors drived with PID
   v5.1 start tunning PID
   v5.2 location computation modified to rely on BNOlocation instead of Atmage computation
   v5.3 debug move accorss path
   v5.4 move accross path rewritten
   v5.5 new wheelcontrol version
   v5.6 BNO mode NDOF non longer used
   v6.0 IR obstacle detections sensors added
   v6.1 IR obstacle detections tunned
   v6.2 IR clear action flags added
   v6.3 hexa action code suppressed  un inpuUDP
   v6.4 modif PID speed and end northalign + check bno iddle at the end of boot
   v6.5 tunning PID after battery modification
   v6.6 modif updateNO
   v6.7 modif pinginit for move backward
   v6.8 correction bug end scan
   v6.9 loosen wheel consistancy condition
   v6.10 gyrorotate limitatio updated
   v6.11 motorControl library instead of motorControl2342L
   v7.0 new wheelcontrol and balance right & left speed
   v7.1 BNO055subsystemV3


*/

//  Version

uint8_t ver[2] = {7, 1};
// uncomment #define debug to get log on serial link
//#define debugScanOn true
//#define debugMoveOn true
//#define debugObstacleOn true
//#define debugLocalizationOn true
//#define debugAcrossPathOn true
//#define debugMagnetoOn true
//#define debugNorthAlign true
//#define debugMotorsOn true
//#define debugPID true
//#define debugWheelControlOn true
//#define wheelEncoderDebugOn true
//#define debugConnection true
//#define debugPowerOn true
//#define debugLoop true
//#define servoMotorDebugOn true
//#define debugGyroscopeOn true
//#define debugGyroscopeL2On true
#include <avr/sleep.h>
#include <avr/power.h>
#define wakePin 17                 // pin used for waking up RX serial2
boolean sleepRequest = false;       // variable to store a request for sleep
#if !defined(PRR) && defined(PRR0)
#define PRR PRR0
#endif

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
//#define IMU true
//#include <ApeRobotSensorSubsytemDefine.h>  // select to use a simple gyroscope
#include <BNO055SubsystemCommonDefine.h>     // or select to use IMU 
#include <NewPing.h>
Servo myservo;  // create servo object to control a servo

//-- comunication --
//#include <SerialNetworkVariable.h>   // needed for communication with esp8266 gateway
//#include <SerialNetworkVoid.h>  // needed for communication with esp8266 gateway

#include <SerialLink.h>
#define gatewayLinkSpeed 38400
SerialLink GatewayLink(gatewayLinkSpeed);   // define the object link to the gateway

// IR obstacle detection
#include <IRObstacleDetection.h>
#include <math.h>
// define IR sensors
#define IRsensor0PIN 32
#define IRsensor1PIN 37
#define IRsensor2PIN 33
#define IRsensor3PIN 35
#define IRsensor4PIN 36
#define IRsensor5PIN 34
#define IRPower1PIN 29 // back
#define IRPower2PIN 28 // front
IRObstacleDetection robotIR;
boolean IrDetectionActive = false;
uint8_t IRSensorsOnMap = 0x00;
uint8_t obstacleSensor = 0x00;
int IRObstacleHeading = 0;
#define IrThreshold 10
#define IrObstacleOn false
#define IrActive true
#define IrInactive false
#define IRSensor 0x01
#define echoSensor 0x02

uint8_t PendingReqRef = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRef = 0xbf; // pending request (if 0x00 none)- copy fo retry
uint8_t PendingReqRefSerial = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRefSerial = 0xbf; // pending request (if 0x00 none)- copy fo retry
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
uint8_t trameNumber = 0;     // frame number to send
uint8_t lastAckTrameNumber = 0;  // last frame number acknowledged by the server
uint8_t pendingAckSerial = 0;    // flag waiting for acknowledged
uint8_t lastReceivedNumber = 0x00;
int retryCount = 0;            // number of retry for sending

//-- General Robot parameters --
#include <ApeRobotCommonDefine.h>
uint8_t rebootPhase = 0x06;
uint8_t rebootDiag = 0x00;

float fLeftTractionDistPerRev =  (PI * fLeftWheelDiameter) ;
float fRightTractionDistPerRev = (PI * fRightWheelDiameter);
float coeffGlissementRotation = 1.;
#define rebootDuration 20000 // delay to completly start arduino
#define rebootBNODuration 2000 // reboot subsytem first step duration
#define rebootESPDuration 40000 // ESP reboot and WIFI maximum duration
#define rebootBNOTimeout 90000 // BNOreboot timeout
#define hornPin 49           // to activate horn

//-- power control --
int uRefMotorVoltage = 1200; // mVolt for maxRPS
//#define power1Pin 53  // power 1 servomoteur
#define power1Value A13  // 9v power input arduino mega
#define power2Value A14  // 5v power input esp8266
#define power3Value A2  // 15v output battery for electronics
#define power4Value A1  // 5v power input servo & echo
#define power5Value A0  // 15v output battery for motors
#define power6Value A3  // 12v power input motors
#define power1LowLimit 750   // minimum centi volt before warning
#define power2LowLimit 475   // minimum centi volt before warning
#define power3LowLimit 1100   // minimum centi volt before warning
#define power4LowLimit 475   // minimum centi volt before warning
#define power5LowLimit 1100   // minimum centi volt before warning
#define power6LowLimit 700   // minimum centi volt before warning
int power1Mesurt = 0;   // current power1 value
int power2Mesurt = 0;   // current power2 value
int power3Mesurt = 0;   // current power3 value
int power4Mesurt = 0;   // current power4 value
int power5Mesurt = 0;   // current power5 value
int power6Mesurt = 0;   // current power6 value
boolean newPowerCycle = true;  // new mesurment cycle each time data are sent to the server

#define encoderPower 48   // used to power on the encoders
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
#include <motorControl.h>  // library for motors control
unsigned int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
unsigned int iRightMotorMaxrpm = iLeftMotorMaxrpm ; // (Value unknown so far - Maximum revolutions per minute for left motor)
//float fMaxrpmAdjustment;  // will be used to compensate speed difference betweeen motors
unsigned int leftMotorPWM = 255;       // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
float SlowPWMRatio = 0.7;              // PWM ratio for a low speed move
#define iLeftSlowPWM leftMotorPWM * SlowPWMRatio        // PMW value to slowdown motor at the end of the run
#define leftRotatePWMRatio 1.0    // 
#define pendingLeftMotor 0      // define pendingAction bit used for left motor
#define maxRPS 1.8 //
#define maxRPS100 maxRPS*100 //
#define minRPS 1.2
#define minRPS100 minRPS*100 //
#define optimalHighStraightSpeed 150  // modif le 23042019 148 vs 152
#define optimalLowStraightSpeed minRPS100 // 130
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, maxRPS * 60, iLeftSlowPWM); // define left Motor
unsigned int rightMotorPWM = 235;      // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
#define iRightSlowPWM rightMotorPWM * SlowPWMRatio  // PMW value to slowdown motor at the end of the run
//#define iRightRotatePWM 170   // PMW value
#define rightRotatePWMRatio 0.9    // 
#define pendingRightMotor 1    // define pendingAction bit used for right motor

Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, maxRPS * 60, iRightSlowPWM); // define right Motor
float leftToRightDynamicAdjustRatio = 1.0;    // ratio used to compensate speed difference between the 2 motors rght PWM = left PWM x ratio
int pulseLenght = 6; // pulse duration
#define slowMoveHolesDuration 12     // target distance expressed in term of numbers of holes under that slow move is required
//-- wheel control --


//#define optimalHighStraightSpeed 200   // unloaded

#define wheelPinInterruptIn 3    // used by sotfware interrupt when rotation reach threshold
#define wheelPinInterruptOut 7    // used by sotfware interrupt when rotation reach threshold
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A7  // analog input right encoder
#define leftWheelId 0          // to identify left wheel Id 
#define rightWheelId 1         // to identify right wheel Id
//boolean endMoveSlowdown = false;
unsigned int iLeftRevSpeed;              // instant left wheel speed
unsigned int iRightRevSpeed;             // instant right wheel speed
#define sizeOfLeftRev leftWheelEncoderHoles // size of the array containing latest revolution wheel speed
#define sizeOfRightRev rightWheelEncoderHoles // size of the array containing latest revolution wheel speed
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
unsigned long BNOprevSentLeftHoles = 0;
unsigned long BNOprevSentRightHoles = 0;
boolean pbSynchro = false;
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
//unsigned int leftIncoderHighValue = 800;  // define value mV above that signal is high
//unsigned int leftIncoderLowValue = 100;  // define value mV below that signal is low
unsigned int leftIncoderHighValue = 500;  // define value mV above that signal is high
unsigned int leftIncoderLowValue = 100;  // define value mV below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
//unsigned int rightIncoderHighValue = 750; // define value mV above that signal is high
//unsigned int rightIncoderLowValue = 200;  // define value mV below that signal is low
unsigned int rightIncoderHighValue = 500; // define value mV above that signal is high
unsigned int rightIncoderLowValue = 100;  // define value mV below that signal is low
//#define delayBetweenEncoderAnalogRead  750 //  micro second between analog read of wheel encoder level
//#define delayMiniBetweenHoles  20  //  delay millis second between 2 encoder holes at the maximum speed  (35)
#define delayMiniBetweenHoles  (1000/(maxRPS*leftWheelEncoderHoles)) //  delay millis second between 2 encoder holes at the maximum speed  
//#define delayMaxBetweenHoles  (1000/(minRPS*leftWheelEncoderHoles))*1.1  //  delay millis second between 2 encoder holes at the minimum speed  
// create wheel control object
boolean encodersStopped = true;
WheelControl Wheels(leftWheelEncoderHoles, leftIncoderHighValue, leftIncoderLowValue, leftAnalogEncoderInput,
                    rightWheelEncoderHoles, rightIncoderHighValue , rightIncoderLowValue, rightAnalogEncoderInput,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    wheelPinInterruptOut, delayMiniBetweenHoles);
//                   wheelPinInterruptOut, delayMiniBetweenHoles, delayMaxBetweenHoles);
volatile boolean encodersToStop = false;   // flag used to delay stopping encoders after stopping motors
//-- move control --
# define bForward  true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
unsigned long iLeftCentiRevolutions;  // nmuber of done revolutions * 100
unsigned long iRightCentiRevolutions; // nmuber of done revolutions * 100
#define leftHoleDistance fLeftWheelDiameter*PI/leftWheelEncoderHoles
#define rightHoleDistance fRightWheelDiameter*PI/rightWheelEncoderHoles
volatile uint8_t wheelIdInterruption = 0xff; // if not 0xff a wheelinterruption has to be analysed

/*
   PID
*/
#include <PID_v1.h>
#define KpRegister 0
#define KiRegister 1
#define KdRegister 2
#define sizeOfKx 3
//double KxLeft[sizeOfKx] = {1.2, 1.5, 0.05}; // {Kp,Ki,Kd}
//double KxRight[sizeOfKx] = {1.2, 1.5, 0.05};  // {Kp,Ki,Kd}
float KxLeft[sizeOfKx] = {0.45, 0.10, 0.12};  // {Kp,Ki,Kd}
float KxRight[sizeOfKx] = {0.45, 0.10, 0.12};  // {Kp,Ki,Kd}
float KxHeading[sizeOfKx] = {1., 0., 2.};  // {Kp,Ki,Kd}
//double KxLeft[sizeOfKx] = {0., 0.80, 0.12};  // {Kp,Ki,Kd}
//double KxRight[sizeOfKx] = {0., 0.80, 0.12};  //{Kp,Ki,Kd}
//double Kx[sizeOfKx] = {1.2, 1.5, 0.05};  // registers that contain PID Kx
#define leftMinOut 0
#define rightMinOut 1
#define leftMaxOut 2
#define rightMaxOut 3
#define leftStartOut 4
#define rightStartOut 5
volatile boolean PIDMode = false;
boolean PIDFirstLoop = true;
#define sizeOfOutlim 6
int outLimit[sizeOfOutlim] = {35, 35, 255, 230, 240, 220}; // {leftMinOut,rightMinOut,leftMaxOut,rightMaxOut, leftStartOut, rightStartOut} % 40 25 243 130 100
//int outLimit[sizeOfOutlim] = {20, 20, 255, 225, 240, 215}; // {leftMinOut,rightMinOut,leftMaxOut,rightMaxOut, leftStartOut, rightStartOut} % 40 25 243 130 100
double leftSetpoint = optimalHighStraightSpeed; // default speed average min max expressed in RPM/100
//double leftSetpoint = minRPS*100; // default speed average min max expressed in RPM/100
volatile double leftInput, leftOutput, headingInput;
volatile double rightSetpoint = leftSetpoint * (fLeftWheelDiameter / fRightWheelDiameter);
volatile double headingSetpoint = 0;
boolean equalSpeed = (leftSetpoint == rightSetpoint);
//double rightSetpoint = maxRPS*100;
double rightInput, rightOutput, headingOutput;
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, KxLeft[KpRegister], KxLeft[KiRegister], KxLeft[KdRegister], DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, KxRight[KpRegister], KxRight[KiRegister], KxRight[KdRegister], DIRECT);
PID headingPID(&headingInput, &headingOutput, &headingSetpoint, KxHeading[KpRegister], KxHeading[KiRegister], KxHeading[KdRegister], DIRECT);
/*
   toDo bit definitions
*/
#define toDoScan 0    // toDo bit for scan request
#define toDoMove 1      // some move  to do
#define toDoRotation 2  // rotation to do
#define toDoStraight 3  // straight move to do
#define toDoBackward 4  // straight move to do is backward
#define toDoClockwise 5 // rotate clockwise
#define toDoAlign 6     // north aligning
#define toDoPingFB 7     // echo ping front back
/*
   toDoDetail bit definitions
*/
#define toDoGyroRotation 0      // rotation based on gyroscope to do
#define toDoMoveAcrossPass 1      // move detail across path
#define toDoAlignRotate 2 //
#define toDoAlignUpdateNO 3 //
#define toDoGetNO 4 //
/*
   waitFlag bit definitions
*/
#define toWait 0        // wait before moving
#define toEndPause 1     // could restart
#define toPause 2     // move to be temporaly paused
/*

*/

int gyroTargetRotation = 0; // expected rotation based on gyroscope
uint8_t gyroRotationRetry = 0;
#define maxGyroRotationRetry 30
int pendingStraight = 0;   // copy of straight distance to be done
int reqAng = 0;          // requested rotation
int reqMove = 0;         // requested move
/*
     use to move across specific pass
*/
int passDistance = 0;
uint8_t passWidth = 0x00;
uint8_t passStartEntryDistance = 0x00;
unsigned int echoToGet = 0;
unsigned int passNO = 0;
uint8_t passLen = 0x00;
#define passMonitorIddle 0x0f
volatile uint8_t passMonitorStepID = passMonitorIddle;
uint8_t lastPassMonitorStepID;
#define passMonitorInterruptBit 7        // interruption has raised from the echo monitor system
#define passMonitorRequestBit 6          // waiting for echo monitor system interruption
#define passSkipStepBit 4                // the last step has been skipped
#define passNumberTrack1 8              //  number of echos to be stored
#define nbTrackedHoles 5
uint8_t passTrack1[2 * passNumberTrack1];   //  contain the echos records
unsigned int passTrackLeftHoles[nbTrackedHoles];         // contain the encoders left holes at checkpoints
unsigned int passTrackRightHoles[nbTrackedHoles];         // contain the encoders right holes at checkpoints
uint8_t passTrackNumber = 0;                 // contain the current number of echos recorded
uint8_t passInterruptBy = 0x00;            // flag to track where the monitor interruption come from
uint8_t passRetCode = 0x00;              // contain the return move return code
uint8_t tracePassMonitorStepID = 0x00;   // bit trace of step id
uint8_t traceInterruptByStepID = 0x00;   // bit trace of step id interruption
/*

*/
uint8_t resumeCount = 0; // nb of echo check before resume move
int movePingInit;     // echo value mesured before moving
int movePingMax;     // echo value mesured before moving to be taken into account
uint8_t rotationType = rotateTypeGyro;     // default rotation type
boolean bSpeedHigh[2] ;        // true means motors [left,right] high PWM false low

//-- accelerometer and magnetometer
// powered by arduino 3.3v
//LSM303 compass;
unsigned long refAccX;
float X;
float Y = 0;
int northOrientation = 0;
int saveNorthOrientation = 0; // last north orientation
int NOBeforeRotation = 0; // keep NO before rotation
int NOAfterRotation = 0;
int NOBeforeMoving = 0; // keep NO before moving straight
int NOAfterMoving = 0; // keep NO before moving straight
int targetAfterNORotation = 0;
int absoluteHeading = 0;
unsigned int northAlignTarget = 0;
boolean northAligned = false;
boolean northAlignedPossibilityChecked  = false;
uint8_t retryAlign = 0x00;
uint8_t stepBNOInitLocation = 0x00;
uint8_t getNorthOrientation = 0xff;

/*
   gyroscope subsytem
*/
boolean gyroCalibrationOk = true; // flag gyroscope calibration true or not
volatile uint8_t inputData[256];           // input zone for data comming from subsystem
uint8_t slaveAddress = robotI2CAddress; // I2c subsytem address
long OutputRobotRequestPinTimer;        //  timer used to reset the request PIN
uint8_t outData[pollResponseLenght] = {slaveAddress, 0x00, 0x00, 0x00};  // output zone for data to be send to the subsystem
#define maxGyroscopeHeadings 100         // maximum number of headings to be stored - must be less than 255
int gyroscopeHeading[maxGyroscopeHeadings]; // pile of last the gyroscope headings
uint8_t gyroscopeHeadingIdx = 0x00;         // current gyroscope heading index
volatile uint8_t gyroUpToDate = 0x00;                // flag defining status of data comming from gyroscope
uint8_t monitSubsystemStatus = 0x00;
uint8_t expectedSubsystemStatus = 0x00;
int prevGyroRotation = 0;
uint8_t gyroInitRotationSens = 0x00;       // rotation clockwise or anti-clockwise

/*
   subsystem internal status
   cf subsystem documentation
*/
uint8_t BNOMode = 0x00;
uint8_t expectedBNOMode = 0x00;
uint8_t BNOCalStat = 0x00;
uint8_t BNOSysStat = 0x00;
uint8_t BNOSysError = 0x00;
/*
  BNORequestedState sttates below
*/
#define BNOIddleState 0xff
#define BNOGetStatus 0x13
#define BNOGotStatus 0x12
#define BNOGetMode 0x11
#define BNOGotMode 0x10
#define BNOGetCompasHeadingState 0x0f
#define BNOGotCompasHeadingState 0x0e
#define BNOGetAbsoluteHeadingState 0x0d
#define BNOGotAbsoluteHeadingState 0x0c
#define BNOGetRelativeHeadingState 0x0b
#define BNOGotRelativeHeadingState 0x0a
#define BNOUpdateHoles 0x09
#define BNOUpdatedHoles 0x08
#define BNOGetHeadingState 0x07
#define BNOGotHeadingState 0x06
#define BNOGetRightState 0x05
#define BNOGotRightState 0x04
#define BNOGetLeftState 0x03
#define BNOGotLeftState 0x02

#define sendBNOState 0x01

#define BNOLoopRetcodeReady 0x00
#define BNOLoopRetcodeWaitingPoll 0x01
#define BNOLoopRetcodeWaitingData 0x02
#define BNOLoopRetcodePollTimeout 0x03
#define BNOLoopRetcodeDataTimeout 0x04

//#define BNOLoopRetcodeWaitStatus 0x07
#define BNOLoopRetcodeWaitInitLocation 0x08
#define BNOLoopRequestedState 0x09
#define BNOLoopRetcodeOverflow 0xff
byte BNOLoopRetcode = 0x00;
boolean BNOLocationToSend = false;
boolean BNORequestSequence = true;                 // set to true to link requests as get all location data - set to fals for getting only one
volatile uint8_t BNORequestedState = BNOIddleState;         // step count when asking for BNO location

uint8_t BNOUpToDateFlag = 0x00;          // flag data up to date
uint8_t compasUpToDate = 0x00;          // flag data up to date
volatile uint8_t pendingPollingResp = 0x00;      // flag waiting for subsystem response
volatile boolean pendingDataResp = false;      // flag waiting for subsystem response
volatile int eventHowMany;
//volatile boolean requestReceived = false;
//volatile boolean eventReceived = false;
uint8_t endMoveToSend = 0x00;
uint8_t endMoveRetCodeToSend = 0x00;

/*
   location data computed by th subsystem
*/
float BNOLeftPosX = 0;
float BNOLeftPosY = 0;
float BNORightPosX = 0;
float BNORightPosY = 0;
float BNOLocationHeading = 0;

/*
   scan control
*/

#define servoPin 31    //  servo motor Pin 
#define echFrontId 0 // identifier for front sonar
#define echBacktId 1 // identifier for back sonar
#define nbPulse 15    // nb of servo positions for a 360° scan
#define echo3 false  // does not exit
#define echo3Alert false  // does not exit
#define echo4 false  // does not exit
#define echo4Alert false  // does not exit
//#define echoPinInterrupt 2 // pin  dedicated to software usage
#define echoPinInterruptIn 2 // a wire between echoPinInterruptIn and echoPinInterruptOut
#define echoPinInterruptOut 6 // 
/*
    echo parameters used to check move possibility before moving forward and rotate
*/
#define echoShift 2                  // servo step used when moving forward
#define echoShiftRotation 14        // servo step size when rotating
#define minimalServoForRotation 20  // minimal servo rotation to check availabilty to rotate
#define delayBetween2Ping 70      // 70ms
/*

*/
boolean toDoEchoFront = true;   // to set echo front on when obstacle detection is running
boolean echoFrontAlertOn = true; // // to set echo front threshold on when obstacle detection is running
boolean toDoEchoBack = true;    // to set echo back on when obstacle detection is running
boolean echoBackAlertOn = true; // to set echo back threshold on when obstacle detection is running
#define echoCycleDuration 0.35  // define cycle in second between 2 triggers when obstacle detection is running
#define echoMonitorCycleDuration 0.07 // define cycle in second between 2 triggers when echo monitoring is running
EchoObstacleDetection echo(echoFront, trigFront, echoBack, trigBack, 0, 0, 0, 0, echoPinInterruptOut);   // create obstacle detection object
boolean obstacleDetectionOn = true;
unsigned int obstacleDetectionCount = 0;
int numStep = 0;     // current scan step number
int nbSteps = 0;     // number of steps to be done for a scan
boolean switchFB = 0;  // switch front back scan
#define nbPulse 15    // nb of servo positions for a 360Â° scan
#define miniServoAngle 0    // corresponding to the lowest value of pulseValue
#define maxiServoAngle 180   // corresponding to the highest value of pulseValue
#define servoAlignedPosition 90 // define the servo angle aligned with robot
#define defaultServoOrientation -1  // depending on servomotor orientation (1 if 0° is at the right of the robot and -1 if 180° is at the right)
//int pulseValue[nbPulse] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165}; // corresponding to 180° split in 15 steps
int pulseValue[nbPulse] = {0, 13, 26, 39, 51, 64, 77, 90, 103, 116, 129, 141, 154, 167, 180}; // corresponding to 180° split in 15 steps
//int pulseValue[nbPulse] = {0, 13, 26, 36, 52, 65, 78, 90, 103, 116, 129, 142, 154, 166, 178}; // corresponding to 180° split in 15 steps
//int pulseValue[nbPulse] = {180, 168, 154, 142, 129, 116, 103, 90,78, 65, 52, 39, 126, 13, 0}; // corresponding to 180° split in 15 steps
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
#define SONAR_NUM 2      // Number of sensors.
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(trigFront, echoFront, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(trigBack, echoBack, MAX_DISTANCE),
};
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
//unsigned long delayCheckSpeed;       // when moving used to regularly compute speed of wheels
unsigned long delayCheckPosition;    // when moving used to regularly compute localization
unsigned long delayPrintSpeed;        // when moving used to regularly print speed for debug
unsigned long timeAffLed;        // used to regularly update LED status
unsigned long serialAliveTimer;  //  for serial link communication
unsigned long serialTimer;       //   for serial link communication
unsigned long checkObstacleTimer;     // to regurarly check obstacle when moving
unsigned long durationMaxEcho = 25000; // maximum scan echo duration en us
unsigned long timeSendInfo;     // to regurarly send information to th server
unsigned long timeAfterStopMotors;     // to avoid I2C perturbations
unsigned long timeBetweenOnOffObstacle;  // use to compute delay between obstacle detection sitch on off
volatile unsigned long timeMotorStarted;          // set to time motors started
unsigned long timerHorn   ;          // set to time horn started
volatile unsigned long pauseSince   ;          // starting time of pause status
unsigned long timePingFB;             // used to delay sending end action after sending echo ping
unsigned long timeGyroRotation;
unsigned long timeCompasRotation;
unsigned long timeSubsystemPolling;   // used for reguraly checking subsystem status
unsigned long timeUpdateNO;           // used for reguraly update north orientation
unsigned long timeUpdateBNOStatus;           // used for reguraly update subsytem status
unsigned long lastSetModeTime;           // used to BNO055 status
unsigned long iddleTimer;               // used to detect robot iddle status
unsigned long lastUpdateBNOMoveTime;    // used to update BNO with robot move
unsigned long lowHighSpeedTimer;       // delat time between change speed of wheels
unsigned long timePassMonitorStarted;
unsigned long timeTraceDataSent;
unsigned long timePID;
unsigned long lastEchoTime;           //
unsigned long timerIRDetection;
unsigned long lastBNOLoopTime = 0;
unsigned long pendingPollingRespTimer = 0;
unsigned long pendingDataRespTimer = 0;
unsigned long requestSystemStatusTime = 0;
unsigned long requestChangeMonitorModeTime = 0;
unsigned long lastUpdateHeading = 0;

#define wheelCheckMoveDelay 500
#define wheelCheckRotateDelay 3000
int wheelCheckDelay = 500;

#if defined debugLoop
unsigned long timeLoop;  // cycle demande heure
#endif
#define delayBetween2Scan 1000  // delay between to scan steps - must take into account the transmission duration
#define delayBetweenScanFB 50  // delay between front and back scan of the same step
#define delayBetweenInfo 5000   // delay before sending new status to the server  
#define delayPowerCheck 100    // delay before next checking of power
#define delayBeforeRestartAfterAbstacle 3000 // delay before taking into account obstacle disappearance
#define delayBetweenCheckPosition 300       // delay between 2 check position
#define delayBetween2NO 500          // delay between 2 NO update
#define delayBetween2BNOStatus 500
#define delayBetweenlastUpdateBNOMoveTime 150
#define delayGyro360Rotation 4000 // mawimum duration for 360° rotation
#define delaylowHighSpeedTimer 20  // maximum delay between 1 wheel still running high speed and 1 starting to slowdown
#define delayBetweenTraceSend 5000
#define delayBetweenIR 80
/*
   delayToStopEnocder delay to take into account mechanical inertia (adjust with video recording) -
   delay between wheels and encoders stop
   to keep not too high to avoid false encoders threshold in case wheel stop in border of a hole
*/
#define delayToStopEnocder 300    // 60 adjusted by tunning
#define delayGyroRotation 200  //
#define maxPauseDuration 10000
#define maxPassMonitorDuration 45000
#define echoPrecision 5
#define echoTolerance 0.2
//-- robot status & diagnostics
volatile uint8_t diagMotor = 0x00;   // bit 0 pb left motor, 1 right moto, 2 synchro motor
uint8_t diagPower = 0x00;   // bit 0 power1, 1 power2, 2 power3 0 Ok 1 Ko
uint8_t diagConnection = 0x07;   // bit 0 pending serial link bit 1 pending I2C bit 2 BNO timeout
uint8_t diagRobot = 0xff;     // to be cleared after complete reboot
uint8_t toDo = 0x00;          // flag kind of move to do
uint8_t toDoDetail = 0x00;          // flag kind of move to do
volatile uint8_t waitFlag = 0xff;     // used to pause and waiting before starting

uint8_t currentMove = 0x00;   // flag kind of current move
uint8_t pendingAction = 0x00; // flag pending action to be started
uint8_t sendInfoSwitch = 0x00;  // flag to switch between different data to send
uint8_t saveCurrentMove = 0x00;   // flag last kind of current move
boolean moveForward;           // flag to indicate if requested move is forward or backward
#define traceBitRequest 7
#define traceBitEncoder 6
#define traceBitPower 5
#define traceBitNO 4
uint8_t appStat = 0xff; // statut code application 00 active ff stopped 8. trace
uint8_t actStat = 0xff; // statut action en cours
uint8_t appTrace = 0x00; // statut trace en cours

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
#define toDoLed 39
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
  GatewayLink.SerialBegin();
  // Serial2.begin(38400); // to communicate with the server through a serial Udp gateway
  Serial.print("Version:");
  Serial.print(ver[0]);
  Serial.print(".");
  Serial.println(ver[1]);
  Serial.print("gateway serial:");
  Serial.println(GatewayLink.SerialNumber());
  int addrEeprom = 0;
  byte valueEeprom;
  valueEeprom = EEPROM.read(addrEeprom);  // id of the arduino stored in eeprom
  Serial.print("ArduinoID=");
  Serial.println(valueEeprom, DEC);
  // **** define gpio mode
  Serial.println("reboot started");
  pinMode(trigFront, OUTPUT);
  pinMode(encoderPower, OUTPUT);
  pinMode(trigBack, OUTPUT);
  digitalWrite(trigFront, LOW);
  digitalWrite(trigBack, LOW);
  pinMode(echoFront, INPUT);
  pinMode(echoBack, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(toDoLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(wheelPinInterruptIn, INPUT_PULLUP);
  pinMode(wheelPinInterruptOut, OUTPUT);
  pinMode(echoPinInterruptIn, INPUT_PULLUP);
  pinMode(echoPinInterruptOut, OUTPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(power1Value, INPUT);
  pinMode(power2Value, INPUT);
  pinMode(wakePin, INPUT);
  digitalWrite(hornPin, LOW);
  digitalWrite(blueLed, HIGH);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, HIGH);
  Serial.println("Start I2C");
  Wire.begin(slaveAddress);                // join i2c bus with address #8
  Wire.onReceive(receiveData); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.println("starting I2C slave mode");
  pinMode(RobotOutputRobotRequestPin, OUTPUT);
  digitalWrite(RobotOutputRobotRequestPin, LOW);
  digitalWrite(toDoLed, LOW);
  Serial.println("I2C ready");
  TurnPIDOn();   // turn PID on
  PrintRobotParameters();
  pinMode(IRPower1PIN, OUTPUT);
  pinMode(IRPower2PIN, OUTPUT);
  digitalWrite(IRPower1PIN, LOW);
  digitalWrite(IRPower2PIN, LOW);
  robotIR.AddSensor(0, IRsensor0PIN, 40, 0, 0, IrThreshold, IrObstacleOn, IrInactive); // front center
  robotIR.AddSensor(1, IRsensor1PIN, 40, 15, 0, IrThreshold, IrObstacleOn, IrInactive); // front left
  robotIR.AddSensor(2, IRsensor2PIN, 40, 15, 2, IrThreshold, IrObstacleOn, IrInactive); // left front
  robotIR.AddSensor(3, IRsensor3PIN, -10, 0, 4, IrThreshold, IrObstacleOn, IrInactive); // back
  robotIR.AddSensor(4, IRsensor4PIN, 40, -15, 6, IrThreshold, IrObstacleOn, IrInactive); // right front
  robotIR.AddSensor(5, IRsensor5PIN, 40, -15, 0, IrThreshold, IrObstacleOn, IrInactive); // front right

}

void loop() {
  delay(1);
  // boolean SubsystemOkForMoving = CheckSubsystemStatus();
#if defined (debugGyroscopeOn)
  //if (!CheckSubsystemStatus() && bitRead(toDo, toDoMove)) {
  // Serial.println("not ready to move");
  //}
#endif
  if (wheelIdInterruption != 0xff)                   // we got wheel encoder interruption
  {
    WheelThresholdReached(wheelIdInterruption);                      // call the threshold analyse
#if defined(debugWheelControlOn)
    Serial.print("wheels interrupt:");
    Serial.println(wheelIdInterruption);
#endif
    //   WheelThresholdReached(wheelIdInterruption);      // interruption treatment
    StopEchoInterrupt();                    // stop obstacles detection
    //  GyroGetHeadingRegisters();
    BNORequestedState = BNOGetRelativeHeadingState;
    wheelIdInterruption = 0xff;                      // clear the flag
  }

  if (IrDetectionActive && millis() > timerIRDetection + delayBetweenIR)
  {
    stateObstacle state = robotIR.CheckObstacle();
    timerIRDetection = millis();
    if (state.change)
    {
#if defined(debugObstacleOn)
      Serial.print("IR Change:");
      Serial.print(state.obstacle);
      Serial.print(" map:");
      Serial.println(IRSensorsOnMap, BIN);
#endif
    }
    if (state.change && state.obstacle)
    {
      IRSensorsOnMap = state.sensorsOnMap;
      IRObstacleHeading = round(robotIR.ObstacleHeading(IRSensorsOnMap) * 180 / PI);
      IRObstacleHeading = (IRObstacleHeading + 360) % 360;
      obstacleSensor = IRSensor;
#if defined(debugObstacleOn)
      Serial.print("stop:");
      Serial.println(IRObstacleHeading);
#endif
      if (bitRead(toDoDetail, toDoAlignRotate))
      {
        InterruptMove(alignEnded, moveKoDueToObstacle);
      }
      else
      {
        InterruptMove(moveEnded, moveKoDueToObstacle);
      }
    }
    if (state.change && !state.obstacle)
    {
#if defined(debugObstacleOn)
      Serial.println("IR RAS");
#endif
    }
  }


#if defined(debugLoop)
  if (millis() - timeLoop > 250)
  {
    debugPrintLoop();                     // for degub only
    timeLoop = millis();
  }
#endif
  CheckEndOfReboot();                      // check for reboot completion
  /*
    if (PIDMode && millis() >= timePID + (delayMiniBetweenHoles / 3))
    {
    ComputePID();
    }
  */
  if (PIDMode)
  {
    ComputePID();
  }
  if (toDo == 0x00 && toDoDetail == 0x00 && pendingAction == 0x00)
  {
    digitalWrite(toDoLed, LOW);
  }
  else
  {
    digitalWrite(toDoLed, HIGH);
  }

  if (rebootPhase == 0x00 && millis() - OutputRobotRequestPinTimer >= minimumDurationBeetwenPolling / 2)
  {
    //digitalWrite(RobotOutputRobotRequestPin, LOW);      // reset the request to the subsystem
  }

  if ( pendingPollingResp == 0x02 && millis() - OutputRobotRequestPinTimer >= 1500 )     // detect timeout
  {
    pendingPollingResp = 0x00;  // to detect timeout
    Serial.println("BNO polling timeout");
    bitWrite(diagConnection, 2, 1); //
  }

  // ***  keep in touch with the server
  // int getSerial = Serial_have_message();  // check if we have received a message
  int getSerial = GatewayLink.Serial_have_message();  // check if we have received a message
  if (getSerial > 0)                      // we got a message
  {
    //   TraitInput(DataInSerial[2]);          // analyze the message
    TraitInput(GatewayLink.DataInSerial[2]);          // analyze the message
  }
  //  if (GatewayLink.PendingReqSerial != 0x00 )           // check if we have a message to send
  //  {
  if (GatewayLink.PendingReqSerial != 0x00 && !pendingDataResp && pendingPollingResp == 0x00 && millis() - timeSendSecSerial > 200)    // check if we have a message to send (or a message currently sending)
  {
    // GatewayLink.DataToSendSerial();                    // send message on the serial link
#if defined(debugConnection)
    Serial.println("send");
#endif
    GatewayLink.DataToSendSerial();                    // send message on the serial link
    timeSendSecSerial = millis();          // reset timer
  }
  if (retryCount >= 5)                    // check retry overflow
  {
    pendingAckSerial = 0x00;               // clear retry flag
    retryCount = 0;                       // clear retry count
  }
  if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 5) {
    GatewayLink.ReSendSecuSerial();                    // re-send data that was not already ack by the server
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
  if (sleepRequest && toDo == 0x00 && toDoDetail == 0x00 && pendingAction == 0x00 && pendingPollingResp == 0x00)
  {
    while (Serial2.available()) {
      int val = Serial2.read();
      Serial.print(val);
      delay(100);
    }
    Serial.print("sleep:");
    Serial.println(digitalRead(wakePin));
    delay(100);
    sleepNow();
  }
  if (millis() - timeReceiveSerial >= 30000)     // 30 seconds without receiving data from the server
  {
    bitWrite(diagConnection, diagConnectionIP, 1);       // conection broken
  }

  if (endMoveToSend == 0x00 && !bitRead(appTrace, traceBitRequest) && (millis() - timeSendInfo >= delayBetweenInfo) && actStat != scan360 && pendingAckSerial == 0x00) // alternatively send status and power to the server
  {
    if (sendInfoSwitch % 6 == 1)
    {
      SendStatus();                 // send robot status to the server
    }
    if (sendInfoSwitch % 6 == 2)
    {
      SendPowerValue();             // send power info to the server
      newPowerCycle = true;
    }
    if (sendInfoSwitch % 6 == 4)
    {
      SendEncodersHolesValues();             //
    }
    if (sendInfoSwitch % 6 == 3)
    {
      SendEncoderValues();
    }
    if (sendInfoSwitch % 6 == 0)
    {
      SendBNOSubsytemStatus();
    }
    if (sendInfoSwitch % 6 == 5)
    {
      SendInternalFlags();
    }
    sendInfoSwitch ++;
    timeSendInfo = millis();
  }
  // *** end loop keep in touch with the server

  // *** power check
  //  if ((millis() - timePowerCheck) >= delayPowerCheck && toDo == 0x00 ) // regularly check power values
  if ((millis() - timePowerCheck) >= delayPowerCheck  ) // regularly check power values
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
    if (bitRead(toDo, toDoScan) == 1  && pendingPollingResp == 0x00)     // check if scan is requested
    {
      if (BNOMode == MODE_COMPASS)
      {
        ScanPosition();
      }
      else {
        //  SetBNOMode(MODE_COMPASS);
        expectedBNOMode = MODE_COMPASS;
      }
    }

    if (CheckSubsystemStatus() && bitRead(toDo, toDoAlign) == 1 && (northAligned == false || bitRead(toDoDetail, toDoAlignUpdateNO)) && pendingPollingResp == 0x00)   // north aligned requested
    {
      if (BNOMode == MODE_COMPASS)
      {
        if (millis() -  timeCompasRotation > 7000 && !leftMotor.RunningMotor() && !rightMotor.RunningMotor())
        {
          if (compasUpToDate == 0x00)
          {
#if defined(debugGyroscopeOn)
            Serial.println("request compas");
#endif
            //   GyroGetHeadingRegisters();
            BNORequestedState = BNOGetCompasHeadingState;
            compasUpToDate = 0x01;
          }
          if (compasUpToDate == 0x02)
          {
#if defined(debugGyroscopeOn)
            Serial.println("got heading");
#endif
            compasUpToDate = 0x00;
            northAlign();
            timeCompasRotation = millis();
          }
        }
      }
      else {
        expectedBNOMode = MODE_COMPASS;
        //  SetBNOMode(MODE_COMPASS);
      }
    }
    if (compasUpToDate == 0x02 && bitRead(toDoDetail, toDoAlignUpdateNO) && !bitRead(toDo, toDoAlign))         // update north orientation requested
    {
#if defined(debugGyroscopeOn)
      Serial.println("send NO");
#endif
      compasUpToDate = 0x00;
      bitWrite(toDoDetail, toDoAlignUpdateNO, 0);
      SendEndAction(actStat, 0x00);
    }
    if (CheckSubsystemStatus() && bitRead(toDo, toDoMove) == 1 && bitRead(toDoDetail, toDoMoveAcrossPass) == 0  && bitRead(waitFlag, toWait) == 0 && pendingPollingResp == 0x00) {   // check if  move is requested
      if (BNOMode != MODE_IMUPLUS ) {
        expectedBNOMode = MODE_IMUPLUS;
        //SetBNOMode(MODE_IMUPLUS);       // set BNO IMU mode
      }
      else if (!bitRead(monitSubsystemStatus, monitGyroStatusBit)) {
        bitWrite(expectedSubsystemStatus, monitMagnetoStatusBit, 1);
      }
      else {
        ComputeTargetLocalization(reqAng, reqMove);
        Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
        appStat = appStat | 0xf0;
      }
    }

    if (CheckSubsystemStatus() && bitRead(toDo, toDoMove) == 1 && bitRead(toDoDetail, toDoMoveAcrossPass) == 1  && bitRead(waitFlag, toWait) == 0 && pendingPollingResp == 0x00) {  // check if  move is requested
      if (BNOMode != MODE_IMUPLUS && passMonitorStepID > 0x01) // NO aligned completed
      {
        //SetBNOMode(MODE_IMUPLUS);    // set BNO IMU mode
        expectedBNOMode = MODE_IMUPLUS;
      }
      else if (BNOMode != MODE_COMPASS && passMonitorStepID == 0x01) // NO aligned to be done
      {
        expectedBNOMode = MODE_COMPASS;
        // SetBNOMode(MODE_COMPASS);    // set BNO IMU mode
      }
      else {
        MoveAcrossPath();
        //      ComputeTargetLocalization(reqAng, reqMove);
        //      Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
        appStat = appStat | 0xf0;
      }
    }
    // *** end of checking resqueted actions

    // *** wheels speed control and localization
    if (bSpeedHigh[leftWheelId] != bSpeedHigh[rightWheelId])
    {
      /*
         one wheel has started to slowdown and the other is still running fast
      */
      if (millis() > lowHighSpeedTimer + delaylowHighSpeedTimer)
      {
        /*
           this must not last too long
        */
        if (bSpeedHigh[leftWheelId])
        {
          leftMotor.AdjustMotorPWM(leftMotorPWM * SlowPWMRatio);
          bSpeedHigh[leftWheelId] = false;
          Wheels.IncreaseThreshold ( leftWheelId, slowMoveHolesDuration );
        }
        if (bSpeedHigh[rightWheelId])
        {
          rightMotor.AdjustMotorPWM(rightMotorPWM * SlowPWMRatio);
          bSpeedHigh[rightWheelId] = false;
          Wheels.IncreaseThreshold ( rightWheelId, slowMoveHolesDuration );
        }
      }
    }
    if ( millis() - delayCheckPosition > delayBetweenCheckPosition  && bitRead(currentMove, toDoStraight) == true && pendingPollingResp == 0x00 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    {
      /* robot is moving straight
         check wheels are correctly running and update robot dynamic location
      */
      delayCheckPosition = millis();  // reset timer
      if (timeMotorStarted != 0 && millis() - timeMotorStarted > wheelCheckDelay && bitRead(diagMotor, diagMotorPbSynchro) == false) // wait for motors to run enough
      {
        CheckMoveSynchronisation();
      }
      //    ComputeNewLocalization(0x00);   // compute dynamic localization
    }
    if ( millis() - delayCheckPosition > delayBetweenCheckPosition  && bitRead(currentMove, toDoRotation) == true && pendingPollingResp == 0x00 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    {
      /* robot is rotating
          check wheels are correctly running
      */
      delayCheckPosition = millis();  // reset timer
      if (timeMotorStarted != 0 && millis() - timeMotorStarted > wheelCheckDelay && bitRead(diagMotor, diagMotorPbSynchro) == false) // wait for motors to run enough
      {
        //        Serial.println(millis() - timeMotorStarted);
        //        Serial.println(bitRead(diagMotor, diagMotorPbSynchro));
        CheckMoveSynchronisation();
      }

    }
#if defined(debugWheelControlOn)
    if (millis() - delayPrintSpeed > 1000 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
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
      Serial.print("end of pause due to timer:");
      Serial.println((millis() - pauseSince));
#endif
      IRSensorsOnMap = 0x00;
      IRObstacleHeading = 0;
      obstacleSensor = echoSensor;
      InterruptMove(moveEnded, moveKoDueToObstacle);
    }
    // *** end of loop checking obstacles

    // *** checking for move completion

    if (CheckSubsystemStatus() && bitRead(toDo, toDoStraight) == true && bitRead(toDoDetail, toDoMoveAcrossPass) == false && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false && pendingPollingResp == 0x00 && stepBNOInitLocation == 0x00)
    { /*      if (BNOMode != MODE_IMUPLUS )
        {
         //        SetBNOMode(MODE_IMUPLUS);
         expectedBNOMode = MODE_IMUPLUS;
         //        delay(250);
        }
        else {
      */
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
      //     }
    }
    // *** end of checking move completion

  }
  if (bitRead(toDo, toDoPingFB) == true && millis() - timePingFB > 500 && pendingPollingResp == 0x00)
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
    //  RequestForPolling();
  }

  if (rebootPhase < 3) {
    byte retcode = BNOLoop();
#if defined(debugGyroscopeL2On)
    if (retcode != BNOLoopRetcode) {
      Serial.print(" prev:0x");
      Serial.print(retcode, HEX);
      Serial.print("new rc:0x");
      Serial.println(BNOLoopRetcode, HEX);
    }
#endif
    BNOLoopRetcode = retcode;
  }
  if (BNORequestedState == BNOIddleState && rebootPhase == 0 && currentMove == 0x00 && gyroUpToDate != 0x01 && !pendingDataResp && pendingPollingResp == 0x00  && stepBNOInitLocation == 0x00 && (millis() - requestSystemStatusTime) > 5000)
  {
    requestSystemStatusTime = millis();
    if (BNOUpToDateFlag % 2 == 0)
    {
      BNORequestedState = BNOGetStatus ;
    }
    if (BNOUpToDateFlag % 2 == 1)
    {
      BNORequestedState = BNOGetMode  ;
    }
    /*
      if (BNOUpToDateFlag % 3 == 2)
      {
      if (BNOMode == MODE_NDOF)
      {
        BNORequestedState =  BNOGetAbsoluteHeadingState;
        //   GetAbsoluteHeading();
      }
      if (BNOMode == MODE_COMPASS)
      {
        //    GetNorthOrientation();
        BNORequestedState = BNOGetCompasHeadingState;
      }
      if (BNOMode == MODE_IMUPLUS)
      {
        //GyroGetHeadingRegisters();
        //        BNORequestedState = BNOGetRelativeHeadingState;
      }
      }
    */
    BNOUpToDateFlag++;
  }
  if ((millis() - lastUpdateBNOMoveTime > delayBetweenlastUpdateBNOMoveTime && BNORequestedState == BNOUpdateHoles))
  {
    UpdateBNOMove();
  }
  else if (bitRead(currentMove, toDoStraight)  && millis() - lastUpdateHeading > 200 ) {
    BNORequestedState = BNOGetHeadingState;
    lastUpdateHeading = millis();
  }
  /*
    if (BNORequestedState == BNOUpdateHoles && pendingPollingResp == 0x02)
    {
    BNORequestedState = BNOUpdatedHoles;
    }
  */

  if (BNORequestedState == BNOIddleState && BNOLocationToSend  && GatewayLink.PendingReqSerial == 0x00 && millis() - timeSendSecSerial > 250 )
  {
#if defined(debugGyroscopeOn)
    Serial.println("send bno loc");
#endif
    SendBNOLocation ();
    BNOLocationToSend = false;
  }

  if (pendingPollingResp == 0x01)           // to be kept at the end of the loop in order to be sure every subsytem response will be taken into account
  {
    //    pendingPollingResp = 0x00;
#if defined(debugGyroscopeOn)
    // Serial.println("clear polling");
#endif
  }
  if (!encodersStopped && encodersToStop && millis() > (timeAfterStopMotors + delayToStopEnocder))
  {
    StopEncoders();
  }
  if (passMonitorStepID << 4  != passMonitorIddle << 4)
  {
    MoveAcrossPath();
  }
  if (bitRead(appTrace, traceBitRequest) && millis() - timeTraceDataSent > delayBetweenTraceSend)
  {
    if (bitRead(appTrace, traceBitNO) && actStat != scan360 && pendingAckSerial == 0x00)
    {
      SendTraceNO();
      if (BNOMode != MODE_COMPASS)
      {
        //      SetBNOMode(MODE_COMPASS);
        expectedBNOMode = MODE_COMPASS;
      }
    }
    else {
      digitalWrite(encoderPower, 1);
      PowerCheck();
      SendTraceData();
      digitalWrite(encoderPower, 0);
    }
    timeTraceDataSent = millis();
  }

  if (endMoveToSend != 0x00 && BNORequestedState == BNOGotLeftState && millis() - timeSendSecSerial > 250) // BNO location has been updated
  {
    delay(50);
    posX = (BNOLeftPosX + BNORightPosX) / 2;
    posY = (BNOLeftPosY + BNORightPosY) / 2;
    alpha = BNOLocationHeading;
    /*
      Serial.print("loc(x,y,H):");
      Serial.print(posX);
      Serial.print(",");
      Serial.print(posY);
      Serial.print(",");
      Serial.println(alpha);
    */
    SendEndAction(endMoveToSend, endMoveRetCodeToSend);
    //    BNORequestedState = BNOIddleState;
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
#if defined (debugMoveOn)
  Serial.print("Move():");
  Serial.print(orientation);
  Serial.print(",");
  Serial.println(lenghtToDo);
#endif
  if (orientation != 0)
  {
    Rotate(orientation, true);
  }

  if (lenghtToDo != 0)
  {
    pendingStraight = lenghtToDo;
  }
}


void MoveForward(int lengthToDo ) {
  //  GyroGetHeadingRegisters();
  delay(10);
  leftSetpoint = optimalHighStraightSpeed;
  rightSetpoint = leftSetpoint;
  wheelCheckDelay = wheelCheckMoveDelay;
  //  NOBeforeMoving = NorthOrientation();
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
  Serial.print("moveToDo:");
  Serial.println(lengthToDo);
  // move Toward
  if (lengthToDo != 0 )
  {

    lentghLeftToDo = abs(lengthToDo);
    lentghRightToDo = abs(lengthToDo);
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, leftMotorPWM, lentghRightToDo, rightMotorPWM);
    if (lengthToDo < 0)
    {
      bLeftClockwise = bForward;
      bRightClockwise = !bForward;
    }
    else {
      bLeftClockwise = !bForward;
      bRightClockwise = bForward;
    }
    if (obstacleDetectionOn)
    {
      boolean doEchoFront;
      boolean doEchoBack;
#if defined(debugObstacleOn)
      Serial.println("check straight move possible");
#endif
      //    int shift = echoShift;
      //   shift =  echoShift * lengthToDo / 100;
      int *minFB = MinEchoFB(90 - echoShift, 2 * echoShift);        // get minimal echo distance front and back
      int minEchoF = *(minFB);
      int minEchoB = *(minFB + 1);
#if defined(debugObstacleOn)
      Serial.print(" F:");
      Serial.print(minEchoF);
      Serial.print(" B:");
      Serial.println(minEchoB);
#endif
      if (lengthToDo < 0)
      {
        moveForward = false;
        movePingInit = -minEchoB;
        echoCurrent = echoBack;
        trigCurrent = trigBack;
        doEchoFront = false;
        doEchoBack = true;
      }
      else
      {
        moveForward = true;
        movePingInit = minEchoF;
        echoCurrent = echoFront;
        trigCurrent = trigFront;
        doEchoBack = false;
        doEchoFront = true;
      }
      trigBoth = false;
      unsigned int distF = 0;
      unsigned int distB = 0 ;
      movePingMax = 0;
      /*
          if movePingInit=0 most likely  the next obstacle beyond sensor limitation
          else check move possibility
      */
      if (movePingInit != 0 && abs((abs(movePingInit) - abs(lengthToDo)) < (frontLenght + securityLenght) * doEchoFront + (backLenght + securityLenght) * doEchoBack))
      {
        movePingMax = abs((abs(movePingInit) - abs((frontLenght + securityLenght) * doEchoFront + (backLenght + securityLenght) * doEchoBack))) * (movePingInit / abs(movePingInit));
#if defined(debugObstacleOn)
        Serial.print("not enough free space to move: ");
        Serial.print(lengthToDo);
        Serial.print(" obstacle: ");
        Serial.print(movePingInit);
        Serial.print(" security: ");
        Serial.print((frontLenght + securityLenght) * doEchoFront + (backLenght + securityLenght) * doEchoBack);
        Serial.print(" max: ");
        Serial.println(movePingMax);
#endif
        if (passMonitorStepID != passMonitorIddle)
        {
          passInterruptBy = 0x02;
          passRetCode = moveKoDueToNotEnoughSpace;
          MoveAcrossPath();
        }
        else {
          EndMoveUpdate(moveEnded, moveKoDueToNotEnoughSpace);
        }
        //     SendEndAction(moveEnded, moveKoDueToNotEnoughSpace);
        bitWrite(pendingAction, pendingLeftMotor, false);
        bitWrite(pendingAction, pendingRightMotor, false);
        bitWrite(currentMove, toDoStraight, false);
        bitWrite(saveCurrentMove, toDoStraight, false);
        lengthToDo = 0;
        return;
      }
      /*
        if (abs(movePingInit) > abs(lengthToDo))
        {
        distF = (abs(movePingInit) - abs(lengthToDo)) * doEchoFront * .9;
        distB = (abs(movePingInit) - abs(lengthToDo)) * doEchoBack * .9;
        }
      */

      EchoServoAlign(servoAlignedPosition, false);
      distF = (frontLenght + securityLenght) * doEchoFront;
      distB = (backLenght + securityLenght) * doEchoBack;
#if defined(debugObstacleOn)
      Serial.print("Starting obstacle distance: ");
      Serial.print(movePingInit);
      Serial.print(" dist echo front: ");
      Serial.print(distF);
      Serial.print(" back: ");
      Serial.println(distB);
#endif
      StartEchoInterrupt(doEchoFront, distF , doEchoBack, distB );
    }
    startMotors();
  }
}





void startMotors()
{
  if (bitRead(currentMove, toDoStraight) || bitRead(toDo, toDoStraight)) {
    PIDMode = true;
    PIDFirstLoop = true;
    BNOLocationHeading = -1;
  }
  else {
    PIDMode = false;
    PIDFirstLoop = true;
  }

  // GyroGetHeadingRegisters();
  if (bitRead(toDo, toDoAlign)) {
    BNORequestedState = BNOGetCompasHeadingState;
  }
  else {
    BNORequestedState = BNOGetRelativeHeadingState;
  }

  digitalWrite(encoderPower, HIGH);
  PowerOnIRSensors();
  delay(100);
  BNORequestSequence = false;
  prevCheckLeftHoles = 0;
  prevCheckRightHoles = 0;
  BNOprevSentLeftHoles = 0;
  BNOprevSentRightHoles = 0;
  currentLeftWheelThreshold = iLeftCentiRevolutions;
  currentRightWheelThreshold = iRightCentiRevolutions;
  if (iLeftCentiRevolutions > 0 && iRightCentiRevolutions > 0)
  {
    bitWrite(pendingAction, pendingLeftMotor, true);
    bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
    Serial.print("currentLeftWheelThreshold:");
    Serial.print(iLeftCentiRevolutions);
    Serial.print(" currentRightWheelThreshold:");
    Serial.println(iRightCentiRevolutions);
#endif
    diagMotor = 0x00;
    saveLeftWheelInterrupt = 0;
    saveRightWheelInterrupt = 0;
    pbSynchro = false;
    diagMotor = 0x00;
    //    digitalWrite(wheelPinInterrupt, LOW);
    attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, FALLING);
    Wheels.StartWheelControl(true, true, currentLeftWheelThreshold , true, true, currentRightWheelThreshold , false, false, 0, false, false, 0);
#if defined(debugMotorsOn)
    Serial.print("currentLeftWheelThreshold:");
    Serial.print(Wheels.GetWheelThreshold(0));
    Serial.print(" currentRightWheelThreshold:");
    Serial.println(Wheels.GetWheelThreshold(1));
#endif
    encodersStopped = false;
    //  int iLeftRevPWM = outLimit[leftStartOut];
    //  int iRightRevPWM = outLimit[rightStartOut];
    float LeftRightRatio = (leftSetpoint / rightSetpoint);
    double startLeftPWM = outLimit[leftStartOut] * LeftRightRatio;
    if (startLeftPWM <= outLimit[leftMinOut])
    {
      startLeftPWM = outLimit[leftMinOut];
    }
    else if (startLeftPWM >= outLimit[leftMaxOut])
    {
      startLeftPWM = outLimit[leftMaxOut];
    }
    double startRightPWM = outLimit[rightStartOut] * LeftRightRatio;
    if (startRightPWM < outLimit[rightMinOut])
    {
      startRightPWM = outLimit[rightMinOut];
    }
    else if (startRightPWM > outLimit[rightMaxOut])
    {
      startRightPWM = outLimit[rightMaxOut];
    }
    rightMotor.RunMotor(bRightClockwise,  startRightPWM);
    leftMotor.RunMotor(bLeftClockwise,  startLeftPWM);

    timeMotorStarted = millis();


    //    ComputePID();
    Horn(true, 100);
  }
}



void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, unsigned int leftMotorPWM, unsigned long iRightDistance, int rightMotorPWM)
{
  // float powerAdustment = float (uRefMotorVoltage) / (3 * map(analogRead(power1Value), 0, 1023, 0, 467)); // speed adjustement to power supply
  float fLeftHoles = (float (iLeftDistance) * leftWheelEncoderHoles / fLeftTractionDistPerRev) ; // Centi-revolutions
  float fRightHoles  = (float (iRightDistance) * rightWheelEncoderHoles / fRightTractionDistPerRev) ; // ms
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
  // bSpeedHigh[leftWheelId] = true;
  // bSpeedHigh[rightWheelId] = true;
#if defined(debugMotorsOn)
  Serial.print("compute L: ");
  Serial.print(iLeftCentiRevolutions);
  Serial.print(" PWM:");
  Serial.print(iLeftRevSpeed);
  Serial.print(" R: ");
  Serial.print(iRightCentiRevolutions);
  Serial.print(" PWM:");
  Serial.println(iRightRevSpeed);
#endif
  if (iLeftCentiRevolutions <= slowMoveHolesDuration + leftWheelEncoderHoles || iRightCentiRevolutions <= slowMoveHolesDuration + rightWheelEncoderHoles ||   passMonitorStepID != passMonitorIddle )
  {
    iLeftRevSpeed = iLeftRevSpeed * SlowPWMRatio;
    iRightRevSpeed = iRightRevSpeed * SlowPWMRatio;
    bSpeedHigh[leftWheelId] = false;
    bSpeedHigh[rightWheelId] = false;
#if defined(debugMotorsOn)
    Serial.println("low speed motion");
#endif
  }
  else
  {
#if defined(debugMotorsOn)
    Serial.println("high speed motion");
#endif
    iLeftRevSpeed = leftMotorPWM ; // revolutions per minute
    iRightRevSpeed = rightMotorPWM * leftToRightDynamicAdjustRatio ; // revolutions per minute
    bSpeedHigh[leftWheelId] = true;
    bSpeedHigh[rightWheelId] = true;
    /*
      if (endMoveSlowdown) {
      iLeftCentiRevolutions = iLeftCentiRevolutions - slowMoveHolesDuration;
      iRightCentiRevolutions = iRightCentiRevolutions - slowMoveHolesDuration;
      }
    */
  }
}


/*
  void ComputeNewLocalization(uint8_t param)  // compute localization according to the wheels rotation
  {
  #if defined(debugLoop)
  Serial.print("comp new loc: 0x");
  Serial.print(param, HEX);
  debugPrintLoop();
  #endif
  unsigned int currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  float deltaAlpha = 0; // modification of robot angles (in degres) since last call of the function
  float deltaLeft = ((float (currentLeftHoles) - saveLeftWheelInterrupt) * PI * fLeftWheelDiameter / leftWheelEncoderHoles); // number of centimeters done by left wheel since last call of the function
  float deltaRight = ((float (currentRightHoles) - saveRightWheelInterrupt) * PI * fRightWheelDiameter / rightWheelEncoderHoles); // number of centimeters done by right wheel since last call of the function
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
      Serial.print("currHL: ");
      Serial.print(currentLeftHoles);
      Serial.print("  prevHL");
      Serial.print(saveLeftWheelInterrupt);
      Serial.print(" ThL");
      Serial.print(Wheels.GetWheelThreshold(leftWheelId));
      Serial.print(" currHR: ");
      Serial.print(currentRightHoles);
      Serial.print(" pervHR");
      Serial.print(saveRightWheelInterrupt);
      Serial.print(" ThR");
      Serial.println(Wheels.GetWheelThreshold(rightWheelId));

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




  #if defined(debugLocalizationOn)
      Serial.print(" alpha: ");
      Serial.println(alpha);
      Serial.print("deltaPosX: ");
      Serial.print(deltaPosX);
      Serial.print(" deltaPosY: ");
      Serial.println(deltaPosY);
  #endif
    }

  }
  if (param == 0x01 && rotationType != rotateTypeGyro)
  {
    deltaAlpha = ((deltaRight + deltaLeft) * 180 / (PI * iRobotWidth )); //
    if (bitRead(currentMove, toDoClockwise) == false )
    {
      deltaAlpha = -deltaAlpha;
    }
    alpha = int((deltaAlpha + alpha)) % 360;
    posX = posRotationCenterX + shiftEchoVsRotationCenter * cos(alpha * PI / 180);    // to take into account the distance beetwen (X,Y) reference and rotation center
    posY = posRotationCenterY + shiftEchoVsRotationCenter * sin(alpha * PI / 180);
    BNOprevSentRightHoles = currentRightHoles;            // to avoid subsystem take into account the rotation when computing the next location
    BNOprevSentLeftHoles = currentLeftHoles;               // to avoid subsystem take into account the rotation when computing the next location
  #if defined(debugLocalizationOn)
    Serial.print("new pos X: ");
    Serial.print(posX);
    Serial.print(" Y: ");
    Serial.print(posY);
    Serial.print(" delta alpha: ");
    Serial.print(deltaAlpha);
    Serial.print(" alpha: ");
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

    //   }
  #if defined(debugLocalizationOn)
    Serial.print("new pos X: ");
    Serial.print(posX);
    Serial.print(" Y: ");
    Serial.print(posY);
    Serial.print(" angle: ");
    Serial.println(alpha);
  #endif
  }
  }
*/

void PrintSpeed()
{
  delayPrintSpeed = millis();
  Serial.print("leftRPS: ");
  Serial.print(Wheels.GetLastTurnSpeed(0));
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(0));
  Serial.print(" rightRPS: ");
  Serial.print(Wheels.GetLastTurnSpeed(1));
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(1));
  Serial.print(" leftHoles: ");
  Serial.print(Wheels.GetCurrentHolesCount(0));
  Serial.print(" rightHoles: ");
  Serial.println(Wheels.GetCurrentHolesCount(1));
}




void AffLed()
{
  if (rebootPhase == 0x00)
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
  else {
    if (rebootPhase == 5)
    {
      digitalWrite(greenLed, 0);
      digitalWrite(blueLed, 0);
      digitalWrite(yellowLed, 0);
      digitalWrite(redLed, 0);
    }
    if (rebootPhase == 4)
    {
      digitalWrite(greenLed, 1);
      digitalWrite(blueLed, 1);
      digitalWrite(yellowLed, 1);
      digitalWrite(redLed, 1);
    }
    if (rebootPhase == 3)
    {
      digitalWrite(greenLed, 0);
      digitalWrite(blueLed, 1);
      digitalWrite(yellowLed, 1);
      digitalWrite(redLed, 1);
    }
    if (rebootPhase == 2)
    {
      digitalWrite(greenLed, 0);
      digitalWrite(blueLed, 0);
      digitalWrite(yellowLed, 1);
      digitalWrite(redLed, 1);
    }
    if (rebootPhase == 1)
    {
      digitalWrite(greenLed, 0);
      digitalWrite(blueLed, 0);
      digitalWrite(yellowLed, 0);
      digitalWrite(redLed, 1);
    }
  }
}


void StopAll()
{
  StopMotors();
  myservo.detach();
  StopEchoInterrupt();
  Wheels.StopWheelControl(true, true, 0, 0);
  GyroStopMonitor();
  digitalWrite(encoderPower, LOW);
  digitalWrite(IRPower1PIN, LOW);
  digitalWrite(IRPower2PIN, LOW);
  detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  //pinMode(wheelPinInterrupt, INPUT);
  appStat = 0xff;       // update robot status
  actStat = 0x00;       // clear action status
  ClearActionFlags();
  pendingAckSerial = 0x00; // clear pending acknowledgement
  timerHorn = 0;          // reset horn timer
  appTrace = 0x00;
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
    Serial.print(" front: ");
    Serial.print(frontL);
  }
  if (backOn == true)
  {
    Serial.print( " back: ");
    Serial.print(backL);
  }
  Serial.println();
  scanNumber = 0;
#endif
  digitalWrite(echoPinInterruptOut, HIGH);
  attachInterrupt(digitalPinToInterrupt(echoPinInterruptIn), obstacleInterrupt, RISING);
  echo.StartDetection(frontOn, backOn, echo3, echo4, echoCycleDuration);
  echo.SetAlertOn(frontOn, frontL, backOn, backL, echo3Alert, 0, echo4Alert, 0);
}


void StopEchoInterrupt()
{
  if (obstacleDetectionOn)
  {
#if defined(debugObstacleOn)
    Serial.println("stop echo interrupt");
#endif
    echo.StopDetection();
    detachInterrupt(digitalPinToInterrupt(echoPinInterruptIn));

    //  pinMode(echoPinInterrupt, INPUT_PULLUP);
    echoCurrent = 0;
    trigCurrent = 0;
    scanNumber = 0;
  }


}

void StopMonitorInterrupt()
{
#if defined(debugObstacleOn)
  Serial.println("stop monitor interrupt");
#endif
  echo.SetMonitorOff();
  echo.StopDetection();
  detachInterrupt(digitalPinToInterrupt(echoPinInterruptIn));

  //  pinMode(echoPinInterrupt, INPUT_PULLUP);
  echoCurrent = 0;
  trigCurrent = 0;
  scanNumber = 0;
  bitWrite(passMonitorStepID, passMonitorInterruptBit, 0); // clear interrupt flag
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
  Serial.print(" check echo number: ");
  Serial.print(echo.GetAlertEchoNumber());
  Serial.print(" dist: ");
  Serial.print(dist);
  Serial.print(" threshold: ");
  Serial.println(echo.GetEchoThreshold(echo.GetAlertEchoNumber()));
#endif

}

void PauseMove()                  //
{
  StopMotors();
  pauseDirectionBackward = bitRead(toDo, toDoBackward);
  //  unsigned int currentLeftWheelThreshold = Wheels.GetWheelThreshold(leftWheelId);
  //  unsigned int currentRightWheelThreshold = Wheels.GetWheelThreshold(rightWheelId);
#if debugObstacleOn
  {
    Serial.print("stop obstacle ");
    Serial.print("encoder left: ");
    Serial.print(currentLeftWheelThreshold);
    Serial.print(" right: ");
    Serial.println(currentRightWheelThreshold);

  }
#endif
  delay(100);
  GyroGetHeadingRegisters();
  int deltaLeft = currentLeftWheelThreshold - Wheels.GetCurrentHolesCount(leftWheelId);
  int deltaRight = currentRightWheelThreshold - Wheels.GetCurrentHolesCount(rightWheelId);
  Serial.print(" left: ");
  Serial.print(deltaLeft);
  Serial.print(" right: ");
  Serial.println(deltaRight);
  pauseLeftWheelInterrupt = max(deltaLeft, 0);
  pauseRightWheelInterrupt = max(deltaRight, 0);
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
#if debugObstacleOn
  {
    Serial.print("pause encoder left: ");
    Serial.print(pauseLeftWheelInterrupt);
    Serial.print(" right: ");
    Serial.print(pauseRightWheelInterrupt);
    Serial.println(" due to obstacle");
  }
#endif
  detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  digitalWrite(encoderPower, LOW);
  //  pinMode(wheelPinInterrupt, INPUT);
  //  delayToStopWheel = millis();
  if (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true)
  {
    Horn(true, 500);
  }
  bitWrite(pendingAction, pendingLeftMotor, false);
  bitWrite(pendingAction, pendingRightMotor, false);

  if (bitRead(currentMove, toDoRotation) == true)
  {
    // ComputeNewLocalization(0x01);
  }
  else {
    // ComputeNewLocalization(0xff);
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
  Serial.print("delta X: ");
  Serial.print(deltaX);
  Serial.print(" Y: ");
  Serial.println(deltaY);
  Serial.print("rotation: ");
  Serial.print(rotation);
  Serial.print(" dist: ");
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
    InterruptMove(moveEnded, moveUnderLimitation);
    //  SendEndAction(moveEnded, moveUnderLimitation);                       //
  }
  SendStatus();
}

void ResumeMoveAfterPause()  // no more obstacle resume moving
{
  bitWrite(diagRobot, diagRobotPause, 0);       // position bit diagRobot
  float lenToDo = (((pauseLeftWheelInterrupt * fLeftWheelDiameter) / leftWheelEncoderHoles + (pauseRightWheelInterrupt * fRightWheelDiameter) / rightWheelEncoderHoles ) * PI / 2);
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
  Serial.print("resume move after pause: ");
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
    bitWrite(pendingAction, pendingLeftMotor, false);
    bitWrite(pendingAction, pendingRightMotor, false);
    bitWrite(currentMove, toDoStraight, false);
    bitWrite(saveCurrentMove, toDoStraight, false);
    if (passMonitorStepID != passMonitorIddle)
    {
      //    passInterruptBy = 0x02;
      passRetCode = moveWheelSpeedInconsistancy;
      //    MoveAcrossPath();
    }
    else {
      EndMoveUpdate(moveEnded, moveUnderLimitation);                       //
    }
  }
  SendStatus();
}
void ComputeTargetLocalization(int rotation, int lenghtToDo)
{
  targetAlpha = alpha + rotation;           // target orientation = current orientation + requested rotation
  targetX = posX + cos(targetAlpha * PI / 180) * lenghtToDo;
  targetY = posY + sin(targetAlpha * PI / 180) * lenghtToDo;
#if defined(debugLocalizationOn)
  Serial.print("target orient: ");
  Serial.print(targetAlpha);
  Serial.print(" x: ");
  Serial.print(targetX);
  Serial.print(" y: ");
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




void StopEncoders()
{
#if debugWheelControlOn
  Serial.println("stopping encoders");
#endif
  encodersToStop = false;
  encodersStopped = true;
  BNORequestSequence = true;
  //passInterruptBy = 0x02;
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  // pinMode(wheelPinInterrupt, INPUT);
  digitalWrite(encoderPower, LOW);
  //  delay(160);                                      // wait a little bit more for robot intertia before requesting heading
  gyroUpToDate = 0x00;
  //GyroGetHeadingRegisters();
  leftWheeelCumulative = leftWheeelCumulative + Wheels.GetCurrentHolesCount(leftWheelId);
  rightWheeelCumulative = rightWheeelCumulative + Wheels.GetCurrentHolesCount(rightWheelId);
  unsigned int leftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int rightHoles = Wheels.GetCurrentHolesCount(rightWheelId);

  float avgRPS = leftHoles / leftWheelEncoderHoles; // turns number
  float duration = (float(millis()) - timeMotorStarted) / 1000;
  avgRPS = (avgRPS / duration);
  timeMotorStarted = 0;
#if defined(debugMoveOn)
  Serial.print(" duration:");
  Serial.print(duration);
  Serial.print("sec >> average left RPS:");
  Serial.print(avgRPS);
  Serial.print(" right:");
  avgRPS = rightHoles / rightWheelEncoderHoles; // turns number
  avgRPS = (avgRPS / duration);
  Serial.println(avgRPS);
#endif
#if defined(debugMoveOn)
  Serial.print("RPM left: ");
  Serial.print(leftWheelId);
  Serial.print(" ");
  Serial.print(Wheels.GetLastTurnSpeed(leftWheelId) * 60);
  Serial.print(" ");
  Serial.println(Wheels.Get2LastTurnSpeed(leftWheelId) * 60);
  Serial.print("Holesleft: ");
  Serial.print(leftHoles);
  Serial.print(" right: ");
  Serial.println(rightHoles);
#endif

  bitWrite(pendingAction, pendingLeftMotor, false);     // clear the flag pending action motor
  bitWrite(pendingAction, pendingRightMotor, false);    // clear the flag pending action motor
  if ( bitRead(currentMove, toDoRotation) == true)      // robot was turning around
  {
    // ComputeNewLocalization(0x01);                       // compute new  robot position
    bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation
    delay(100);
    //      NOAfterRotation = NorthOrientation();
    if (bitRead(toDo, toDoStraight) == false && bitRead(toDoDetail, toDoGyroRotation) == 0)
    {
      if (pbSynchro) {
        if (passMonitorStepID != passMonitorIddle)
        {
          passRetCode = moveWheelSpeedInconsistancy;
          MoveAcrossPath();
        }
        else {
          EndMoveUpdate(moveEnded, moveWheelSpeedInconsistancy);
        }
      }
      else
      {
        if (passMonitorStepID == passMonitorIddle) // standard move
        {
          EndMoveUpdate(moveEnded, 0x00);
        }
      }
      BNORequestedState = BNOGetHeadingState;
    }
  }
  else
  {
    //UpdateBNOMove();
    BNORequestedState = BNOUpdateHoles;
    // ComputeNewLocalization(0xff);                       // compute new  robot position
    bitWrite(currentMove, toDoStraight, false) ;        // clear flag todo straight
    actStat = moveEnded;                                      // status move completed
    if (!bitRead(toDoDetail, toDoGyroRotation)) {
      toDo = 0x00;                                         // clear flag todo
    }

    uint8_t retCode = 0x00;
    // bitWrite(diagMotor, diagMotorPbEncoder, 0);
    //  bitWrite(diagMotor, diagMotorPbLeft, 0);
    //   bitWrite(diagMotor, diagMotorPbRight, 0);
    delay(100);
    //    NOAfterMoving = NorthOrientation();
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
      Serial.print("pb level encoder minLeft; ");
      Serial.print(Wheels.GetMinLevel(leftWheelId));
      Serial.print(" min right: ");
      Serial.print(Wheels.GetMinLevel(rightWheelId));
      Serial.print( "max left: ");
      Serial.print(Wheels.GetMaxLevel(leftWheelId));
      Serial.print( "max right: ");
      Serial.println(Wheels.GetMaxLevel(rightWheelId));
#endif
    }
    if (retCode == 0 && pbSynchro)
    {
      if (passMonitorStepID != passMonitorIddle)
      {
        //      passInterruptBy = 0x02;
        passRetCode = moveWheelSpeedInconsistancy;
        //     MoveAcrossPath();
      }
      else {
        EndMoveUpdate(moveEnded, moveWheelSpeedInconsistancy);
      }
    }
    else
    {
      if (passMonitorStepID != passMonitorIddle)
      {
        MoveAcrossPath();
      }
      else {
        if (endMoveToSend == 0x00)
        {
          EndMoveUpdate(moveEnded, 0x00);
        }

      }
    }
  }

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

/*
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
*/
void initNorthAlign(unsigned int alignTarget)
{
  northAlignTarget = alignTarget;
  northAligned = false;
  northAlignedPossibilityChecked = false;
  iddleTimer = millis();
  retryAlign = 0;
#if defined(debugNorthAlign)
  Serial.print("north align:");
  Serial.println(northAlignTarget);
#endif
  //       targetAfterNORotation= ((int) alpha + reqN) % 360;
  targetAfterNORotation = 0;
}

#define minAlign northAlignPrecision
#define maxAlign 360-northAlignPrecision
void northAlign()
{ // North rotation anti-clockwise positive    - robot rotation clockwise positive
  if (!northAlignedPossibilityChecked && bitRead(toDoDetail, toDoAlignRotate))
  {
    northAlignedPossibilityChecked = true;
    int toToCheck = OptimizeNorthAlign(northOrientation, northAlignTarget);
#if defined(debugNorthAlign)
    Serial.print("toToCheck:");
    Serial.print(northOrientation);
    Serial.print(" ");
    Serial.print(northAlignTarget);
    Serial.print(" ");
    Serial.println(toToCheck);
#endif

    if (CheckRotationAvaibility(toToCheck))
    {
#if defined(debugNorthAlign)
      Serial.println("possible");
#endif
    }
    else {
#if defined(debugNorthAlign)
      Serial.println("not possible:");
#endif
      InterruptMove(alignEnded, moveKoDueToNotEnoughSpace);
      actStat = alignEnded; // align required
      northAligned = false;
      bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
      bitWrite(toDoDetail, toDoAlignRotate, 0);
      bitWrite(toDoDetail, toDoGyroRotation, 0);
      EndMoveUpdate(alignEnded, moveKoDueToNotEnoughSpace);
      StopMagneto();
      return;
    }
  }
  if (!bitRead(toDoDetail, toDoAlignRotate))
  {
    if (bitRead(toDoDetail, toDoAlignUpdateNO))
    {
#if defined(debugNorthAlign)
      Serial.print("send end NO:");
      Serial.println(northOrientation);
#endif
      //     SendStatus();
      saveNorthOrientation = northOrientation;
      SendEndAction(requestUpdateNO, 0x00);
      getNorthOrientation = 0x00;
      bitWrite(toDo, toDoAlign, 0);
      bitWrite(toDoDetail, toDoAlignUpdateNO, 0);
      StopMagneto();
      timeSendInfo = millis();
    }
    return;
  }
  iddleTimer = millis();
  compasUpToDate = 0x00;
  actStat = aligning; // align required
  saveNorthOrientation = northOrientation;
#if defined(debugNorthAlign)
  Serial.print("NO:");
  Serial.println(northOrientation);
#endif
  int  alignShift =  OptimizeNorthAlign(saveNorthOrientation, northAlignTarget) ;
#if defined(debugNorthAlign)
  Serial.print("NO:");
  Serial.print(northOrientation);
  Serial.print(" retry:");
  Serial.print(retryAlign);
  Serial.print(" shift:");
  Serial.println(alignShift);
#endif
  if (retryAlign > 30) {
    actStat = alignEnded; // align required
    northAligned = false;
    bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
    bitWrite(toDoDetail, toDoGyroRotation, 0);
    bitWrite(toDoDetail, toDoAlignRotate, 0);
    EndMoveUpdate(alignEnded, rotationKoToManyRetry);
    StopMagneto();
    return;
  }
  if ((abs(alignShift) >= minAlign) && (abs(alignShift) <= maxAlign))
  {
#if defined(debugNorthAlign)
    Serial.print(" rotate: ");
    Serial.println(alignShift);
#endif
    //      toDo = 0x00;
    //     bitWrite(toDo, toDoAlign, 1);       // position bit toDo
    appStat = appStat & 0x1f;
    int rot = 0;
    if (alignShift >= 0 && alignShift <= 180)
    {
      bLeftClockwise = true;
      bRightClockwise = bLeftClockwise;
      rot = alignShift;
    }
    if (alignShift < 0 && alignShift >= - 180)
    {
      bLeftClockwise = false;
      bRightClockwise = bLeftClockwise;
      rot = alignShift;
    }
    if (alignShift >= 180)
    {
      bLeftClockwise = true;
      bRightClockwise = bLeftClockwise;
      rot = 180 - alignShift;
    }
    if ( alignShift <= - 180)
    {
      bLeftClockwise = false;
      bRightClockwise = bLeftClockwise;
      rot = 360 + alignShift;
    }
    rot = rot * .9;
    if (abs(alignShift) >= 2 * minRotEncoderAbility)
    {
      if (CheckRotationAvaibility(rot))
      {
        Rotate(rot, false);
      }
      else {
        actStat = alignEnded; // align required
        northAligned = false;
        bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
        bitWrite(toDoDetail, toDoAlignRotate, 0);
        bitWrite(toDoDetail, toDoGyroRotation, 0);
        EndMoveUpdate(alignEnded, moveKoDueToNotEnoughSpace);
        StopMagneto();
        return;
      }
    }
    else
    {
      pulseMotors(pulseLenght);
      delay(500);     // to let time for inertia
    }
    delay(1000);
    retryAlign++;
    if (passMonitorStepID == 0x01) {
      timePassMonitorStarted = millis();
    }
  }

  else
  {
    delay(1000);
    //     compass.read();  // double check
    //     saveNorthOrientation = int(compass.heading());
    //   unsigned int  alignShift = (saveNorthOrientation + 360 - northAlignTarget) % 360;
    if ((abs(alignShift) >= minAlign) && (abs(alignShift) <= maxAlign))
    {
      retryAlign++;
      if (passMonitorStepID == 0x01) {
        timePassMonitorStarted = millis();
      }
    }
    else
    {
      actStat = alignEnded; // align required
      northAligned = true;
      if (targetAfterNORotation != 0)         // case rotation based on NO
      {
        alpha = targetAfterNORotation;        // set orientation to the expectation
      }
      bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
      bitWrite(toDoDetail, toDoAlignRotate, 0);
      bitWrite(toDoDetail, toDoGyroRotation, 0);
      EndMoveUpdate(alignEnded, 0x00);
      if (!bitRead(toDoDetail, toDoAlignUpdateNO))
      {
        //    StopMagneto();
        bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 1);
        bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 0);
      }
    }
  }
}

int GyroscopeRotate()
/*
   rotate the robot using the gyroscope as a reference
*/
{
  gyroUpToDate = 0x00;
  if (bitRead(monitSubsystemStatus, monitGyroStatusBit) == 1)
  {
    //  GyroGetHeadingRegisters();
    BNORequestedState = BNOGetRelativeHeadingState;
    //   timeGyroRotation = millis();
    gyroRotationRetry++;         // count the number of time we tried to reach the target heading
#if defined(debugGyroscopeOn)
    Serial.print("gyro retry:");
    Serial.print(gyroRotationRetry);
    Serial.print(" toDo: 0x");
    Serial.print(toDo);
    Serial.print("  0x");
    Serial.println(toDoDetail);
#endif
    boolean aligned = false;
    actStat = gyroRotating; //
    // gyroTargetRotation=rotation;
    // delay(100);

    if (gyroInitRotationSens != 0x00 && abs(gyroTargetRotation) >= minRotGyroAbility)     // first step
    {
      gyroInitRotationSens = 0x00;
      prevGyroRotation = gyroTargetRotation;
      OptimizeGyroRotation(gyroTargetRotation);
#if defined(debugGyroscopeOn)
      Serial.print("first gyro rotation step: ");
      Serial.print(gyroTargetRotation);
      Serial.print(" toDo: 0x");
      Serial.print(toDo);
      Serial.print("  0x");
      Serial.println(toDoDetail);
#endif
      if (gyroTargetRotation < 0)
      {
        gyroTargetRotation = (360 + gyroTargetRotation) % 360;
      }

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
    int gyroH = gyroscopeHeading[gyroscopeHeadingIdx];  // get current heading
    int rotation = 0;
    //    gyroH = (360 - gyroH)%360;        // change to anti clockwiseint totation=0;
#if defined(debugGyroscopeOn)
    Serial.print("gyro relativeheading: ");
    Serial.println(gyroH);
#endif
    if ((gyroTargetRotation > 0 && gyroTargetRotation < 90) && (gyroH > 270 && gyroH < 360))
    {
#if defined(debugGyroscopeOn)
      Serial.print("case 1 over 0: ");
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
#if defined(debugGyroscopeOn)
      Serial.print("case 1 over 0: ");
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
#if defined(debugGyroscopeOn)
      Serial.print("new rotation: ");
      Serial.println(rotation);
#endif
    }
    prevGyroRotation = rotation;
#if defined(debugGyroscopeOn)
    Serial.print("gyro: ");
    Serial.println(rotation);
#endif
    if (abs(rotation) < minRotationTarget)
    {
#if defined(debugGyroscopeOn)
      Serial.print("end gyro: ");
      Serial.println(rotation);
#endif
      aligned = true;
      bitWrite(toDo, toDoRotation, 0);       // clear bit toDo
      bitWrite(toDoDetail, toDoGyroRotation, 0);       // clear bit toDo
      alpha = alpha + gyroH;
      ComputeAngleAndPositionVSGyro(gyroH);
      EndMoveUpdate(moveEnded, 0x00);
      return 0;
    }
    else
    {
      if (abs(rotation) < minRotEncoderAbility)
      {
#if defined(debugGyroscopeOn)
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
        pulseMotors(pulseLenght);
      }
      else
      {
#if defined(debugGyroscopeOn)
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
    GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
#if defined(debugGyroscopeOn)
    Serial.print("start gyro monitor: ");
    Serial.println(!bitRead(toDo, toDoBackward), HEX);
#endif
    return -1;
  }
}


void debugPrintLoop()
{
  Serial.print(" currentMove: 0x");
  Serial.print(currentMove, HEX);
  Serial.print(" todo: 0x");
  Serial.print(toDo, HEX);
  Serial.print(" tododetail: 0x");
  Serial.print(toDoDetail, HEX);
  Serial.print(" pending Straight: ");
  Serial.print(pendingStraight);
  Serial.print(" waitFlag: 0x");
  Serial.print(waitFlag, HEX);
  Serial.print(" diagRobot: 0x");
  Serial.print(diagRobot, HEX);
  Serial.print(" pendingAction: 0x");
  Serial.print(pendingAction, HEX);
  Serial.print(" pendingBNO: ");
  Serial.print(pendingPollingResp);
  Serial.print(" passMonitorStepID: 0x");
  Serial.print(passMonitorStepID, HEX);
  Serial.print(" BNOUpToDateFlag:0x");
  Serial.print(BNOUpToDateFlag, HEX);
  Serial.print(" pendingPollingResp:0x");
  Serial.print(pendingPollingResp, HEX);
  Serial.print(" BNORequestedState: 0x");
  Serial.print(BNORequestedState, HEX);
  Serial.print(" BNOLoopRetcode: 0x");
  Serial.println(BNOLoopRetcode, HEX);

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

void OptimizeGyroRotation(int rotation)
{
  boolean check = (gyroRotationRetry < 2);
  if (abs(rotation) > 3 * maxInertialRotation)
  {
    if (rotation >= 0)
    {
      int newRotation = rotation - maxInertialRotation;
      Rotate(newRotation, check);
#if defined(debugGyroscopeOn)
      Serial.print("gyro: ");
      Serial.print( newRotation);
      Serial.print(" delay ");
      Serial.print( delayGyroRotation);
#endif
    }
    else
    {
      int newRotation = rotation + maxInertialRotation;
      Rotate(newRotation, check);
#if defined(debugGyroscopeOn)
      Serial.print("gyro: ");
      Serial.println( rotation + maxInertialRotation);
#endif
    }
  }
  else
  {
    if (rotation >= 0)
    {
      int newRotation = max(round(rotation * 0.8), minRotEncoderAbility);
      Rotate(newRotation, check);
#if defined(debugGyroscopeOn)
      Serial.print("gyro: ");
      Serial.println( newRotation);
#endif
    }
    else
    {
      int newRotation = min(round(rotation * 0.8), -minRotEncoderAbility);
      Rotate(newRotation, check);
#if defined(debugGyroscopeOn)
      Serial.print("gyro: ");
      Serial.println(newRotation);
#endif
    }
  }
}

void PrintRobotParameters()
{
  Serial.print("Wheel diameter left: ");
  Serial.print(fLeftWheelDiameter);
  Serial.print(" right: ");
  Serial.println(fRightWheelDiameter);
  Serial.print("Width front: ");
  Serial.print(iRobotFrontWidth);
  Serial.print(" back: ");
  Serial.println(iRobotWidth);
  Serial.print("min rot encoder ability: ");
  Serial.println(minRotEncoderAbility);
  Serial.print("wheel holes left: ");
  Serial.print(leftWheelEncoderHoles);
  Serial.print(" right: ");
  Serial.println(rightWheelEncoderHoles);
  Serial.print("shift echo vs rotation center: ");
  Serial.print(shiftEchoVsRotationCenter);
  Serial.print(" front / back: ");
  Serial.println(shiftEchoFrontBack);
  Serial.print("lenght front: ");
  Serial.print(frontLenght);
  Serial.print(" back: ");
  Serial.println(backLenght);
  Serial.print("security lenght: ");
  Serial.println(securityLenght);
  Serial.print("min to be done dist / cm: ");
  Serial.print(minDistToBeDone);
  Serial.print(" rot / deg: ");
  Serial.print(minRotToBeDone);
  Serial.print(" rot target / deg: ");
  Serial.println(minRotationTarget);
  Serial.print("max inertial rotation: ");
  Serial.println(maxInertialRotation);
}

boolean CheckRotationAvaibility(int rotation)
{
#if defined(debugObstacleOn)
  Serial.println("CheckRotationAvaibility");
#endif
  if (abs(rotation) <= 3 * minRotToBeDone) // no check > IR sensors will detect obtsacle
  {
    return true;
  }
  int signRotation = 1;
  if (rotation < 0)
  {
    signRotation = -1;
  }

  if (rotation >= -90 && rotation <= 90)
  {
    int *minFB = MinEchoFB(90, signRotation * max(abs(rotation), minimalServoForRotation));         // get minimal echo distance front and back
    //        int *minFB = MinEchoFB(90, rotation);         // get minimal echo distance front and back
    //   int minEchoF = minFB[0];
    //   int minEchoB = minFB[1];
    int minEchoF = *(minFB);
    int minEchoB = *(minFB + 1);
    if (minEchoF + securityLenght < iRobotFrontDiag || minEchoB + securityLenght < iRobotBackDiag)
    {
#if defined(debugObstacleOn)
      Serial.print("rotation not possible:");
      Serial.print(rotation);
      Serial.print(" F:");
      Serial.print(minEchoF);
      Serial.print(" B:");
      Serial.println(minEchoB);
#endif
      return false; // not enough space to rotate

    }
    else
    {
#if defined(debugObstacleOn)
      Serial.println("rotation possible");
#endif
      return true;
    }
  }
  if (rotation > 90)
  {
    int *minFB = MinEchoFB(90, 90);              // get minimal echo distance front and back
    //  int minEchoF = minFB[0];
    // int minEchoB = minFB[1];
    int minEchoF = *(minFB);
    int minEchoB = *(minFB + 1);
    if (minEchoF + securityLenght < iRobotFrontDiag || minEchoB + securityLenght < iRobotBackDiag)
    {
#if defined(debugObstacleOn)
      Serial.print("rotation not possible:");
      Serial.print(rotation);
      Serial.print(" F:");
      Serial.print(minEchoF);
      Serial.print(" B:");
      Serial.println(minEchoB);
#endif
      return false; // not enough space to rotate

    }
    else
    {
      int *minFB = MinEchoFB(180 - rotation, 90 - rotation);          // get minimal echo distance front and back
      //     minEchoF = minFB[0];
      //     minEchoB = minFB[1];
      minEchoF = *(minFB);
      minEchoB = *(minFB + 1);
      if (minEchoF + securityLenght < iRobotBackDiag || minEchoB + securityLenght < iRobotFrontDiag)
      {
#if defined(debugObstacleOn)
        Serial.print("rotation not possible:");
        Serial.print(rotation);
        Serial.print(" F:");
        Serial.print(minEchoF);
        Serial.print(" B:");
        Serial.println(minEchoB);
#endif
        return false; // not enough space to rotate

      }
      else {
#if defined(debugObstacleOn)
        Serial.println("rotation possible");
#endif
        return true;
      }

    }
  }
  if (rotation < -90)
  {
    int *minFB = MinEchoFB(90, -90);              // get minimal echo distance front and back
    //   int minEchoF = minFB[0];
    //  int minEchoB = minFB[1];
    int minEchoF = *(minFB);
    int minEchoB = *(minFB + 1);
    if (minEchoF + securityLenght < iRobotFrontDiag || minEchoB + securityLenght < iRobotBackDiag)
    {
#if defined(debugObstacleOn)
      Serial.print("rotation not possible:");
      Serial.print(rotation);
      Serial.print(" F:");
      Serial.print(minEchoF);
      Serial.print(" B:");
      Serial.println(minEchoB);
#endif
      return false; // not enough space to rotate

    }
    else
    {
      int *minFB = MinEchoFB( rotation + 180, 90 - rotation);       // get minimal echo distance front and back
      //     minEchoF = minFB[0];
      //     minEchoB = minFB[1];
      minEchoF = *(minFB);
      minEchoB = *(minFB + 1);
      if (minEchoF + securityLenght < iRobotBackDiag || minEchoB + securityLenght < iRobotFrontDiag)
      {
#if defined(debugObstacleOn)
        Serial.print("rotation not possible:");
        Serial.print(rotation);
        Serial.print(" F:");
        Serial.print(minEchoF);
        Serial.print(" B:");
        Serial.println(minEchoB);
#endif
        return false; // not enough space to rotate

      }
      else {
#if defined(debugObstacleOn)
        Serial.println("rotation possible");
#endif
        return true;
      }

    }
  }
  /*
    else {
    if (minEchoF + securityLenght < iRobotBackDiag || minEchoB + securityLenght < iRobotFrontDiag)
    {
    #if defined(debugObstacleOn)
    Serial.println("rotation not possible");
    #endif
    return false; // not enough space to rotate

    }
    else
    {
    #if defined(debugObstacleOn)
    Serial.println("rotation possible");
    #endif
    return true;
    }
    }
  */
}

int * MinEchoFB(int fromPosition, int rotation)
{
  /*
    This function return the minimum echo distance got by a scan from 'fromPostion' taken into account the rotation
  */
  static int MinFB[2];       // minimum echo distance checked before moves
  MinFB[0] = 9999;
  MinFB[1] = 9999;
  int rotationSens = rotation / abs(rotation);
  //maxiServoAngle
  //miniServoAngle
  int signRotation = 1;
  if (rotation < 0)
  {
    signRotation = -1;
  }
  rotation = rotation % 180;
  int servoRotation = rotation ;

#if defined(debugObstacleOn)
  Serial.print("from: ");
  Serial.print(fromPosition);
  Serial.print(" rotation: ");
  Serial.print(rotation);
  Serial.print(" servo rotation: ");
  Serial.println(servoRotation);
#endif
  if ((rotation >= 0 && rotation <= 90) || rotation < -90)
  {
    for (int servoOrientation = fromPosition;
         (abs(servoOrientation) <= min(maxiServoAngle, abs((fromPosition + echoShiftRotation * signRotation / 2 + servoRotation))));
         servoOrientation = servoOrientation + signRotation * echoShiftRotation )
    {
      EchoServoAlign(servoOrientation, false);
      int echoF = PingFront();
      delay(100);
      echoF = max(echoF, PingFront());
      if (echoF != 0 && echoF < MinFB[0])
      {
        MinFB[0] = echoF;
      }
      delay(100);
      int echoB = PingBack();
      echoB = max(echoB, PingBack());
      if (echoB != 0 && echoB < MinFB[1])
      {
        MinFB[1] = echoB;
      }
    }
  }
  else {

    for (int servoOrientation = fromPosition;
         ( servoOrientation >= max(miniServoAngle, (fromPosition - echoShiftRotation / 2 + servoRotation )));
         servoOrientation = servoOrientation - echoShiftRotation )
    {
      EchoServoAlign( servoOrientation, false);
      int echoF = PingFront();
      if (echoF != 0 && echoF < MinFB[0])
      {
        MinFB[0] = echoF;
      }
      delay(100);
      int echoB = PingBack();
      if (echoB != 0 && echoB < MinFB[1])
      {
        MinFB[1] = echoB;
      }
    }
  }

#if defined(debugObstacleOn)
  Serial.print("minEchoF: ");
  Serial.print(MinFB[0]);
  Serial.print(" minEchoB: ");
  Serial.println(MinFB[1]);
#endif

  return MinFB;
}
void InterruptMove(uint8_t action, uint8_t retCode)
{
#if defined (debugObstacleOn)
  Serial.println("interrupt");
  debugPrintLoop();
#endif
  if (passMonitorStepID != passMonitorIddle)
  {
    passRetCode = retCode;
  }
  else {
    StopMotors();
    timeAfterStopMotors = millis();
    bitWrite(pendingAction, pendingLeftMotor, false);     // clear the flag pending action motor
    bitWrite(pendingAction, pendingRightMotor, false);    // clear the flag pending action motor
    if (bitRead(currentMove, toDoStraight) == true)      // robot was moving straight
    {
      encodersToStop = true;                               // to keep wheel ecoders running a little bit
    }
    if ( bitRead(currentMove, toDoRotation) == true)      // robot was turning around
    {
      delay(500);                                      // wait a little for robot intertia
      Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
      detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
      digitalWrite(encoderPower, LOW);
      gyroUpToDate = 0x00;
      GyroGetHeadingRegisters();
      timeAfterStopMotors = millis();
      bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation
      delay(100);
    }
    bitWrite(waitFlag, toPause, 0);       // clear wait & pause flag
    bitWrite(waitFlag, toEndPause, 0);
    bitWrite(waitFlag, toWait, 0);
    StopEchoInterrupt();                    // stop obstacles detection
    resumeCount = 0;                      // clear count
    toDo = 0x00;                                         // clear flag todo
    actStat = action;                                      // status move completed
    EndMoveUpdate(actStat, retCode);                       // move not completed due to obstacle
  }
}

int AngleDiff(int alpha, int beta)
{
  int delta = ((360 - abs(alpha - beta + 360) % 360) % 360);
  return delta;
}
int OptimizeNorthAlign(int currentNO, int targetNO)
{
  int delta = AngleDiff(currentNO, targetNO);
  int rotation = 0;
  if (delta > 180)
  {
    rotation = AngleDiff(targetNO, currentNO);
  }
  else
  {
    rotation = -delta;
  }
  if (rotation > 0)
  {
    rotation = rotation + northAlignPrecision / 2;
  }
  else {
    rotation = rotation - northAlignPrecision / 2;
  }
  return rotation;
}
void StopMotors()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  bitWrite(pendingAction, pendingLeftMotor, LOW) ;
  bitWrite(pendingAction, pendingRightMotor, LOW) ;
  PIDMode = false;
  PIDFirstLoop = false;
  BNORequestSequence = true;
  //delay(100);
}
void EndMoveUpdate(uint8_t action, uint8_t retCode)
{
  endMoveToSend = action;
  endMoveRetCodeToSend = retCode;
  BNORequestedState = BNOUpdateHoles;                // to update last encoders data
  toDo = 0x00;
  toDoDetail = 0x00;                                         // clear flag todo
  pendingStraight = 0x00;
  digitalWrite(IRPower1PIN, LOW);
  digitalWrite(IRPower2PIN, LOW);
  digitalWrite(encoderPower, LOW);
}
void PowerOnIRSensors()
{
  if (bitRead(currentMove, toDoRotation) || bitRead(toDo, toDoRotation) || bitRead(toDoDetail, toDoAlignRotate))
  {
    digitalWrite(IRPower1PIN, HIGH);
    robotIR.StartSensor(0);
    robotIR.StartSensor(1);
    robotIR.StartSensor(2);
    robotIR.StartSensor(4);
    robotIR.StartSensor(5);
  }
  if (bitRead(currentMove, toDoStraight) || bitRead(toDo, toDoStraight)  || bitRead(currentMove, toDoMoveAcrossPass) || bitRead(toDo, toDoMoveAcrossPass))
  {
    digitalWrite(IRPower1PIN, HIGH);
    robotIR.StartSensor(0);
    robotIR.StartSensor(1);
    robotIR.StartSensor(2);
    robotIR.StartSensor(4);
    robotIR.StartSensor(5);
  }
  if (bitRead(currentMove, toDoBackward) || bitRead(toDo, toDoBackward))
  {
    digitalWrite(IRPower2PIN, HIGH);
    robotIR.StartSensor(3);
  }
  delay(500);
  IrDetectionActive = true;
  stateObstacle state = robotIR.CheckObstacle();
}

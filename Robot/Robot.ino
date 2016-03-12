String Version = "RobotV1";
// uncomment #define debug to get log on serial link
//#define debugScanOn true
#define debugMoveOn true
//#define debugObstacleOn true
#define debugLocalizationOn true
#define debugMotorsOn true
#define debugWheelControlOn true
#define debugConnection true
//#define debugPowerOn true
#define wheelEncoderDebugOn true
#include <Servo.h>  // the servo library use timer 5 with atmega
#include <math.h>
#include <EEPROM.h>  // 
//#include <Stepper.h>
#include <SoftwareSerial.h>
#include <EchoObstacleDetection.h>
#include <WheelControl.h>
#include <Wire.h>        // for accelerometer
#include <LSM303.h>     // for accelerometer
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
boolean reboot = true;
int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iRightMotorMaxrpm = iLeftMotorMaxrpm ; // (Value unknown so far - Maximum revolutions per minute for left motor)
float fMaxrpmAdjustment;  // will be used to compensate speed difference betweeen motors
int iLeftWheelDiameter = 64; //(in mm - used to measure robot moves)  65mn a vid
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
unsigned int iLeftTractionDistPerRev =  (PI * iLeftWheelDiameter) ;
unsigned int iRightTractionDistPerRev = (PI * iRightWheelDiameter);
int iRobotWidth = 455; // distance beetwen the 2 wheels mm
float coeffGlissementRotation = 1.;
#define frontLenght  35 // from echo system cm
#define backLenght  12 // from echo system  cm
#define securityLenght 30 // minimal obstacle distance  cm
#define rebootDuration 10000 // delay to completly start arduino
#define hornPin 49

//-- power control --
int uRefMotorVoltage = 1200; // mVolt for maxRPM
//#define power1Pin 53  // power 1 servomoteur
#define power1Value A13  // 9v power for motors
#define power2Value A14  // 5v power for arduino mega
//#define power3Value A15  // 12V power for esp8266 and components
#define power1LowLimit 700   // minimum centi volt before warning
#define power2LowLimit 480   // minimum centi volt before warning
//#define power3LowLimit 750  // minimum centi volt before warning
int power1Mesurt = 0;   // current power1 value
int power2Mesurt = 0;   // current power2 value
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
#define iLeftSlowPWM 125        // PMW value to slowdown motor at the end of the run
#define iLeftRotatePWM 160        // PMW value to slowdown motor at the end of the run
#define pendingLeftMotor 0      // define pendingAction bit used for left motor
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPWM); // define left Motor
#define iRightSlowPWM iLeftSlowPWM   // PMW value to slowdown motor at the end of the run
#define iRightRotatePWM iLeftRotatePWM   // PMW value to slowdown motor at the end of the run
#define pendingRightMotor 1    // define pendingAction bit used for right motor
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPWM); // define right Motor
float leftToRightDynamicAdjustRatio = 1.0;    // ratio used to compensate speed difference between the 2 motors
//-- wheel control --
#define wheelPinInterrupt 3    // used by sotfware interrupt when rotation reach threshold
#define leftWheelEncoderHoles 8  // number of holes of the encoder wheel
#define rightWheelEncoderHoles leftWheelEncoderHoles // number of holes of the encoder wheel
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A10  // analog input right encoder
#define leftWheelId 0           // to identify left wheel Id 
#define rightWheelId 1         // to identify right wheel Id
unsigned int iLeftRevSpeed;              // instant left wheel speed
unsigned int iRightRevSpeed;             // instant right wheel speed
#define sizeOfLeftRev 8 // size of the array containing latest revolution wheel speed
#define sizeOfRightRev 8 // size of the array containing latest revolution wheel speed
unsigned long saveLeftWheelInterrupt = 0;           // copy of previous number of interrupts for the left encoder
unsigned long saveRightWheelInterrupt = 0;          // copy of previous number of interrupts for the right encoder
long leftWheeelCumulative = 0;            // cumulative count of the left holes used for dynamic speed adjustment
long rightWheeelCumulative = 0;           // cumulative count of the right holes used for dynamic speed adjustment
unsigned int rightMotorPWM = 215;      // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
unsigned int leftMotorPWM = 235;       // default expected robot PMW  must be < 255 in order that dynamic speed adjustment could increase this value
// to adjust low and high value set leftWheelControlOn true, rotate left wheel manualy and read on serial the value with and wihtout hole
// must fit with the electonic characteristic
// use calibrate() to set the incoder high and low value
/*
 *
 * unsigned int leftIncoderHighValue = 300;  // define value above that signal is high
unsigned int leftIncoderLowValue = 112;  // define value below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
unsigned int rightIncoderHighValue = 610; // define value above that signal is high
unsigned int rightIncoderLowValue = 487;  // define value below that signal is low
 */
unsigned int leftIncoderHighValue = 850;  // define value above that signal is high
unsigned int leftIncoderLowValue = 190;  // define value below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
unsigned int rightIncoderHighValue = 850; // define value above that signal is high
unsigned int rightIncoderLowValue = 190;  // define value below that signal is low
//#define delayBetweenEncoderAnalogRead  750 //  micro second between analog read of wheel encoder level
#define delayMiniBetweenHoles  40  //  delay millis second between 2 encoder holes at the maximum speed
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
#define toDoMove 1      // some move  to do
#define toDoRotation 2  // rotation to do
#define toDoStraight 3  // straight move to do
#define toDoBackward 4  // straight move to do is backward
#define toDoClockwise 5 // rotate clockwise
#define toDoAlign 6     // north aligning
#define toWait 0        // wait before moving
#define toEndPause 1     // could restart
#define toPause 2     // move to be temporaly paused
#define minDistToBeDone 30  // mn
#define minRotToBeDone 5  // degree
#define scanning 102   // 0x66
#define scanEnded 103  // 0x67
#define moving 104     // 0x68
#define moveEnded 105  // 0x69
#define aligning 106  // 0x6a
#define alignEnded 107 //0x6b

int pendingStraight = 0;   // copy of straight distance to be done
int reqAng = 0;          // requested rotation
int reqMove = 0;         // requested move
uint8_t resumeCount = 0; // nb of echo check before resume move
int movePingInit;     // echo value mesured before moving

//-- accelerometer and magnetometer
// powered by arduino 3.3v
LSM303 compass;
unsigned long refAccX;
float X;
float Y = 0;
int saveNorthOrientation = 0; // last north orientation
// compass calibration to be made
/*
#define compassMin1 -3076
#define compassMin2 -3378
#define compassMin3 -2844
#define compassMax1 +2654
#define compassMax2 +2189
#define compassMax3 +2921

#define compassMin1 -1296
#define compassMin2 -1926
#define compassMin3 -2900
#define compassMax1 +1532
#define compassMax2 +389
#define compassMax3 -2210
*/
#define compassMin1 -661
#define compassMin2 -1792
#define compassMin3 -3132
#define compassMax1 +1502
#define compassMax2 +497
#define compassMax3 -2560

//-- scan control --
#define servoPin 28    //  stepper
#define toDoScan 0    // toDo bit for scan request
#define echoFront 18  // arduino pin for mesuring echo delay for front 
#define echFrontId 0 //
#define trigFront  22     // arduino pin for trigerring front echo
#define echoBack  19   // arduino pin for mesuring echo delay for back 
#define trigBack  23    // arduino pin for trigerring back echo
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
int numStep = 0;     // current scan step number
int nbSteps = 0;     // number of steps to be done for a scan
boolean switchFB = 0;  // switch front back scan
#define nbPulse 15    // nb of servo positions for a 360Â° scan
int pulseValue[nbPulse] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165}; // corresponding to 360Â° split in 15 steps
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
#define delayBetween2Scan 1500  // delay between to scan steps - must take into account the transmission duration
#define delayBetweenScanFB 700  // delay between front and back scan of the same step
#define delayBetweenInfo 5000   // delay before sending new status to the server  
#define delayPowerCheck 5000    // delay before next checking of power
#define delayBeforeRestartAfterAbstacle 5000 // delay before taking into account obstacle disappearance

//-- robot status & diagnostics
uint8_t diagMotor = 0x00;   // bit 0 pb left motor, 1 right moto, 2 synchro motor
uint8_t diagPower = 0x00;   // bit 0 power1, 1 power2, 2 power3 0 Ok 1 Ko
uint8_t diagConnection = 0x01;   // bit 0 pending serial link
uint8_t diagRobot = 0xff;     // to be cleared after complete reboot
uint8_t toDo = 0x00;          // flag kind of move to do
uint8_t waitFlag = 0xff;     // used to pause and waiting before starting

uint8_t currentMove = 0x00;   // flag kind of current move
uint8_t pendingAction = 0x00; // flag pending action to be started
uint8_t sendInfoSwitch = 0x00;  // flag to switch between different data to send
uint8_t saveCurrentMove = 0x00;   // flag last kind of current move
boolean moveForward;           // flag to indicate if requested move is forward or backward
uint8_t appStat = 0xff; // statut code application 00 active ff stopped 1er demi octet mouvement 2eme demi scan
uint8_t actStat = 0xff; // statut action en cours

//-- space localizarion --
float alpha = 0; // current orientation
long posX = 0;   // current x position
long posY = 0;   // current y position
float targetAlpha = 0; // orientation target position after moving
long targetX = 0;  // x target position after moving
long targetY = 0;// x target position after moving
float deltaPosX = 0;  // incremental x move
float deltaPosY = 0;  // incremental y move
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
  pinMode(trigFront, OUTPUT);
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
  pinMode(wheelPinInterrupt, OUTPUT);
  pinMode(hornPin, OUTPUT);
  digitalWrite(hornPin, LOW);
  digitalWrite(blueLed, HIGH);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, HIGH);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  /*
    compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
    -3076, -3378, -2796
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +2654, +2189, +2991
  };
  */
  compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
    compassMin1, compassMin2, compassMin3
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    compassMax1, compassMax2, compassMax3
  };
  //     compass.read();
  //    refAccX=abs(compass.a.x)*1.1;
  EchoServoAlign();                               // adjust servo motor to center position
}

void loop() {

  CheckEndOfReboot();                      // check for reboot completion

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
    bitWrite(diagConnection, 1, 0);       // conection broken
  }

  if (millis() - timeSendInfo >= delayBetweenInfo && actStat != 0x66)  // alternatively send status and power to the server
  {
    if (sendInfoSwitch % 5 != 4)
    {
      SendStatus();                 // send robot status to the server
    }
    if (sendInfoSwitch % 5 == 4)
    {
      SendPowerValue();             // send power info to the server
    }
    sendInfoSwitch = sendInfoSwitch + 1;
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
  if (reboot == false && appStat != 0xff)         //  wait reboot complete to act
  {

    // *** checking resqueted actions
    if (bitRead(toDo, toDoScan) == 1)          // check if for scan request
    {
      ScanPosition();
    }
    if (bitRead(toDo, toDoMove) == 1 && bitRead(waitFlag, toWait) == 0) {      // check if for move request
      //  Serial.println("move");
      ComputeTargetLocalization(reqAng, reqMove);
      Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
      appStat = appStat | 0xf0;
    }
    // *** end of checking resqueted actions

    // *** wheels speed control and localization

    if ( millis() - delayCheckPosition > 250  && bitRead(currentMove, toDoStraight) == true && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    { // Update robot position
      delayCheckPosition = millis();  // reset timer
      if ((millis() - timeMotorStarted) > 500 )   // wait for motors to run enough
      {
        //       CheckMoveSynchronisation();
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
    if (bitRead(waitFlag, toPause) == 1 && (millis() - checkObstacleTimer) > 750 )    // robot in pause status
    {
      CheckNoMoreObstacle();                    // does the obstacle disappeared ?
      checkObstacleTimer = millis();            // reset timer
      if ( bitRead(waitFlag, toEndPause) == 1 ) // no more obstacle
      {
        resumeCount++;
        if (resumeCount > 3)                    // "double check" before reume move
        {
          bitWrite(waitFlag, toPause, 0);       // clear wait & pause flag
          bitWrite(waitFlag, toEndPause, 0);
          bitWrite(waitFlag, toWait, 0);
          resumeCount = 0;                      // clear count
          ResumeMove();                         // resume move


        }
      }

    }

    // *** end of loop checking obstacles

    // *** checking for move completion

    if (bitRead(toDo, toDoStraight) == true && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false)
    {
      if ( bitRead(currentMove, toDoRotation) == true )  // end of first move step (rotation) move straight to be done
      {
#if defined(debugMoveOn)
        Serial.println("end rotation still straight move todo");
#endif
      }
      MoveForward(pendingStraight) ;
    }

    // *** end of checking move completion

  }
  if (timerHorn != 0 && timerHorn < millis())
  {
    Horn(false, 0);
  }
}
void TraitInput(uint8_t cmdInput) {     // wet got data on serial
  //  Serial.println("serialInput");
  bitWrite(diagConnection, 0, 0);       // connection is active
  timeReceiveSerial = millis();         // reset check receive timer
  switch (cmdInput) {                   // first byte of input is the command type
    case 0x73: // commande s means stop
      Serial.println("cmd stop");
      StopAll();
      break;
    case 0x78: // commande x menas start
      Serial.println("cmd start");
      appStat = 0x00;
      break;
    case 0x77: // commande w means calibrate wheels
      Serial.println("calibrate");
      CalibrateWheels();
      break;
    case 0x65: // commande e server request for robot status
      if (actStat != 0x66)
      {
        SendStatus();                                     // send robot status to server
      }
      break;
    case 0x49: // commande I init robot postion
      posX = DataInSerial[4] * 256 + DataInSerial[5]; // received X cm position on 3 bytes including 1 byte for sign
      if (DataInSerial[3] == 0x2d) {                  // check sign + or -
        posX = -posX;
      }
      posY = DataInSerial[7] * 256 + DataInSerial[8]; //received Y cm position on 3 bytes including 1 byte for sign
      if (DataInSerial[6] == 0x2d) {                 // check sign + or -
        posY = -posY;
      }
      alpha = DataInSerial[10] * 256 + DataInSerial[11]; //received orientation (degre) on 3 bytes including 1 byte for sign

      if (DataInSerial[9] == 0x2d) {                 // check sign + or -
        alpha = -alpha;
      }
      deltaPosX = 0;
      deltaPosY = 0;
      currentLocProb = DataInSerial[13];   // localisation probability
      SendStatus();
#if defined(debugConnection)
      Serial.print("init X Y Alpha:");
      Serial.print(posX);
      Serial.print(" ");
      Serial.print(posY);
      Serial.print(" ");
      Serial.println(alpha);
#endif
      break;
    case 0x2b: // commande + means scan 360
      if (appStat != 0xff)
      {
        appStat = appStat & 0xf1;
        Serial.println("Scan");
        pulseNumber = 0;
        InitScan(nbPulse, 0);
        actStat = 0x66;
        bitWrite(toDo, toDoScan, 1);       // position bit toDo scan
      }
      //      SendRFNoSecured();
      break;
    case 0x67: // commande goto X Y position
      {

        int reqX = DataInSerial[4] * 256 + DataInSerial[5];
        moveForward = true;           // goto forward
        if (DataInSerial[3] == 0x2d) {
          reqX = -reqX;
        }

        int reqY = DataInSerial[7] * 256 + DataInSerial[8];
        if (DataInSerial[6] == 0x2d) {
          reqY = -reqY;
        }
#if defined(debugConnection)
        Serial.print("goto X");
        Serial.print(reqX);
        Serial.print(" Y:");
        Serial.println(reqY);
#endif
        targetX = reqX;
        targetY = reqY;
        toDo = 0x00;
        bitWrite(toDo, toDoMove, 1);       // position bit toDo move
        appStat = appStat & 0x1f;
        actStat = 0x68; // moving
        ResumeMove();
        break;
      }
    case 0x6d: // commande m means move
      if (appStat != 0xff)
      {
        appStat = appStat & 0x1f;
        actStat = 0x68; // moving

        toDo = 0x00;
        bitWrite(toDo, toDoMove, 1);       // position bit toDo move
        reqAng = DataInSerial[4] * 256 + DataInSerial[5];

        if (DataInSerial[3] == 0x2d) {
          reqAng = -reqAng;
        }
        if (reqAng != 0)
        {
          bitWrite(toDo, toDoRotation, 1);
        }
#if defined(debugConnection)
        Serial.print("Move: ");
        Serial.print(reqAng);
#endif
        reqMove = DataInSerial[7] * 256 + DataInSerial[8];
        if (DataInSerial[6] == 0x2d) {
          reqMove = -reqMove;
        }
        if (reqMove > 0)
        {
          bitWrite(toDo, toDoStraight, true);
          bitWrite(toDo, toDoBackward, false);   // to go forward
        }
        if (reqMove < 0)
        {
          bitWrite(toDo, toDoStraight, true);
          bitWrite(toDo, toDoBackward, true);  // to go backward
        }
#if defined(debugConnection)
        Serial.print(" ");
        Serial.println(reqMove);
        Serial.print("todo 0x");
        Serial.print(toDo, HEX);
        Serial.print(" waitflag 0x");
        Serial.println(waitFlag, HEX);
#endif
      }
      break;
    case 0x45: // N north align
      {
        unsigned int reqN = DataInSerial[3] * 256 + DataInSerial[4];

#if defined(debugConnection)
        Serial.print("north align:");
        Serial.println(reqN);
#endif
        northAlign(reqN);
        break;
      }
    case 0x61: // we receiver a ack from the server
      lastAckTrameNumber = DataInSerial[3];
#if defined(debugConnection)
      Serial.print("ack: ");
      for (int i = 0; i < 5; i++)
      {
        Serial.print(DataInSerial[i], HEX);
        Serial.print(":");
      }
      Serial.println();

      //      Serial.print(":");
      Serial.print(lastAckTrameNumber);
      Serial.print(":");
      Serial.println(trameNumber);
#endif
      if (lastAckTrameNumber <= trameNumber )
      {
        pendingAckSerial = 0x00;
      }
      break;
    default:

      Serial.print("commande recue: ");
      Serial.println(cmdInput, HEX);

  }
}

void IncServo(int sens) {   // increase servo motor position depending on sens value
  valAng = IncPulse(sens ); // compute servo motor value depending on sens value
  myservo.write(valAng);    // move servo motor
  delay(750);              // wait to be sure the servo reach the target position
}
int IncPulse(int sens) { // compute servo motor value depending on sens
  pulseNumber = pulseNumber + sens;    //
  return (pulseValue[(pulseNumber) % nbPulse]) ;

}
int PingFront() {               // ping echo front
  int cm;
  unsigned long lecture_echo;   //
  unsigned long time1;
  unsigned long time2;
  unsigned long deltaT;
  digitalWrite(trigFront, LOW);  // ajoute le 24/12/2015 a avlider
  delayMicroseconds(2);      //
  digitalWrite(trigFront, HIGH);
  delayMicroseconds(15); // 10 micro sec mini
  time1 = micros();
  digitalWrite(trigFront, LOW);
  lecture_echo = pulseIn(echoFront, HIGH, durationMaxEcho);
  time2 = micros();
  deltaT = (time2 - time1);
  if (deltaT >= durationMaxEcho)
  {
    deltaT = 0;
  }
  else
  {
    deltaT = deltaT * 0.905;
  }

  cm = deltaT / 58  ;
  //Serial.println(cm);

  return (cm);

}
int PingBack() {
  int cm;
  unsigned long lecture_echo;   //
  unsigned long time1;
  unsigned long time2;
  unsigned long deltaT;
  digitalWrite(trigBack, LOW);  // ajoute le 24/12/2015 a avlider
  delayMicroseconds(2);      //
  digitalWrite(trigBack, HIGH);

  delayMicroseconds(15); // 10 micro sec mini
  time1 = micros();
  digitalWrite(trigBack, LOW);
  lecture_echo = pulseIn(echoBack, HIGH, durationMaxEcho);
  time2 = micros();
  deltaT = (time2 - time1);
  if (deltaT >= durationMaxEcho)
  {
    deltaT = 0;
  }
  else
  {
    deltaT = deltaT * 0.905;
  }
  Serial.println();
  Serial.print("echo:");
  Serial.print(lecture_echo);
  Serial.println();

  Serial.print("deltaT Back:");
  Serial.println(time2 - time1);
  cm = deltaT / 58  ;
  return (cm);

  //  }
}
void InitScan(int nbS, int startingOrientation)    // int scan
{
  numStep = 0;                                     // init current number step
  nbSteps = nbS;                                   // init number of servo steps to do
  scanOrientation = startingOrientation;           // init starting servo position
  myservo.attach(servoPin);                       // attaches the servo motor
  myservo.write(pulseValue[scanOrientation]);     // set the servo motor to the starting position
  delay(1000);                                    // wait long enough for the servo to reach target
}

void ScanPosition() {

  // if ((millis() - timeScanFront) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 )
  if ((millis() - timeScanBack) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 && pendingAckSerial == 0x00)
  {

    timeScanFront = millis();
    switchFB = 1;
    distFSav = ScanOnceFront(numStep);
  }
  if ((millis() - timeScanFront) > delayBetweenScanFB && numStep <= abs(nbSteps) - 1 && switchFB == 1 && pendingAckSerial == 0x00)
    //  if ((millis() - timeScanBack) > delayBetweenScanFB && numStep <= abs(nbSteps) - 1 && switchFB == 1 )
  {
    timeScanBack = millis();
    switchFB = 0;
    distBSav = ScanOnceBack(numStep);
    SendScanResultSerial(distFSav, distBSav);
    IncServo(1);
    numStep = numStep + 1;
  }

  if (pendingAckSerial == 0x00 && numStep >= abs(nbSteps)  )
  {
    Serial.println("scanEnd");
    bitWrite(toDo, toDoScan, 0);       // position bit toDo scan
    timeScanBack = millis();
    switchFB = 0;
    myservo.detach();
    SendEndScan();
  }
}

int ScanOnceFront(int numStep)
{
  int distF = PingFront();

  AngleRadian = (pulseValue[(pulseNumber) % nbPulse] - pulseValue[0]) * coefAngRef;
  AngleDegre = (AngleRadian / PI) * 180;
  Serial.print(" ");
  Serial.print(AngleDegre);
  Serial.print(" dist front:");
  Serial.println(distF);
  return distF;

}
int ScanOnceBack(int numStep)
{
  int distB = PingBack();
  trameNumber = trameNumber + 1;
  AngleRadian = (pulseValue[(pulseNumber) % nbPulse] - pulseValue[0]) * coefAngRef;
  AngleDegre = AngleRadian / PI * 180;
  // Serial.print(AngleRadian);
  //  Serial.print(";");
  Serial.print(" ");
  Serial.print(AngleDegre);
  Serial.print(" dist back:");
  Serial.print(distB);
  Serial.println();

  return distB;

}

void SendEndScan()
{

  int northOrientation = NorthOrientation();
  actStat = 0x67; //" scan completed
  PendingDataReqSerial[0] = actStat;
  PendingDataReqSerial[1] = appStat;
  PendingDataReqSerial[3] = 0x00;
  PendingDataReqSerial[4] = uint8_t(northOrientation / 256);
  PendingDataReqSerial[5] = uint8_t(northOrientation);
  PendingDataReqSerial[6] = 0x00;
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = 0x00;
  PendingDataReqSerial[9] = 0x00;
  PendingDataLenSerial = 0x0a;

  PendingReqSerial = PendingReqRefSerial;
  myservo.write(pulseValue[(nbPulse + 1) / 2]); // remise au centre
  delay(1000);
  // myservo.detach();  // attaches the servo on pin 4 to the servo object
  // digitalWrite(power1Pin, LOW);
  numStep = 0;

}
void Move(int orientation, int lenghtToDo)
{


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
  // EchoServoAlign();
  // trigCurrent = trigFront;
  // trigBoth = true;         // need to scan front and back during rotation for obstacle detection
  //  echoCurrent = echoFront; // start echo front
  // StartEchoInterrupt(1, 1);
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
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, iLeftRotatePWM, lentghRightToDo, iRightRotatePWM);
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

void MoveForward( int lengthToDo) {
  EchoServoAlign();
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
    Serial.print("Starting obstacle distance:");
    Serial.println(movePingInit);
    StartEchoInterrupt(doEchoFront, doEchoBack);
    startMotors();
  }
}

unsigned int  RotationCalculation(int orientation) {
  unsigned int distToDo = ((iRobotWidth * PI / 360) * orientation);
  Serial.print("distRot:");
  Serial.println(distToDo);
  return (distToDo);
}

void SendScanResultSerial (int distF, int distB)   // send scan echo data to the server
{
  retryCount = 00;
  PendingDataReqSerial[0] = 0x01;
  PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  PendingDataReqSerial[2] = 0x46; //"F" front
  PendingDataReqSerial[3] = uint8_t(distF / 256);
  PendingDataReqSerial[4] = uint8_t(distF);
  PendingDataReqSerial[5] = 0x42; //"B" back
  PendingDataReqSerial[6] = uint8_t(distB / 256);
  PendingDataReqSerial[7] = uint8_t(distB);
  PendingDataReqSerial[8] = 0x4; //"A" angle
  PendingDataReqSerial[9] = uint8_t(AngleDegre / 256); //1er octet contient les facteurs de 256
  PendingDataReqSerial[10] = uint8_t(AngleDegre); //2eme octets contient le complement - position = Datareq2*256+Datareq3
  PendingDataReqSerial[11] = uint8_t(trameNumber % 256);
  PendingDataReqSerial[12] = 0x00;
  PendingDataReqSerial[13] = 0x00;
  PendingDataReqSerial[14] = 0x00;
  PendingDataLenSerial = 0x0f;                      // data len
  pendingAckSerial = 0x01;
  PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();                   // init timer used to check for server ack
}


void PowerCheck()
{
  timePowerCheck = millis();                       // reset timer
  //  Serial.print("power1:");
  unsigned int power1 = analogRead(power1Value); // read power1 analog value and map to fit real voltage
  power1 = analogRead(power1Value);              // read twice to get a better value
  power1Mesurt = (power1 * float (1010) / 1023);  // map to fit real voltage
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
  power2Mesurt = (power2 * float (975) / 1023);  // map to fit real voltage
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
    myservo.detach();
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

  bitWrite(pendingAction, pendingLeftMotor, true);
  bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
  Serial.print("reqSpeed:");
  Serial.print(leftMotorPWM);
  Serial.print(" ");
  Serial.println(rightMotorPWM);
#endif
  diagMotor = 0x00;
  digitalWrite(wheelPinInterrupt, LOW);
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
  iRightRevSpeed = rightMotorPWM * leftToRightDynamicAdjustRatio; // revolutions per minute
#if defined(debugMotorsOn)
  Serial.print("compute L:");
  Serial.print(iLeftCentiRevolutions);
  Serial.print(" R:");
  Serial.println(iRightCentiRevolutions);
#endif
}

void CheckMoveSynchronisation()       // check that the 2 wheels are rotating at the same pace
{
  float deltaMove = abs(Wheels.GetCurrentHolesCount(0) - Wheels.GetCurrentHolesCount(1));
  bitWrite(diagMotor, 3, 0);       // position bit diagMotor
  boolean pbSynchro = false;
  if (Wheels.GetCurrentHolesCount(0) != 0)
  {
    if ( (deltaMove > 2 && deltaMove / Wheels.GetCurrentHolesCount(0) > 0.05)  )
    {
      pbSynchro = true;
    }
  }
  else
  {
    pbSynchro = true;
  }
  if (pbSynchro == true)
  {

    leftMotor.StopMotor();
    rightMotor.StopMotor();
    StopEchoInterrupt(true, true);                    // stop obstacles detection
    Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
    detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
    bitWrite(diagMotor, 3, 1);       // position bit diagMotor
    Serial.print("move synchro pb:");
    Serial.print(deltaMove);
    Serial.print(" ");
    Serial.println(abs(deltaMove / Wheels.GetCurrentHolesCount(0)));
    StopEchoInterrupt(true, true);                    // stop obstacles detection
    delay(200);                                       // wait a little for robot intertia
    Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
    detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
    //    bitWrite(pendingAction, pendingLeftMotor) == true
    bitWrite(pendingAction, pendingLeftMotor, false);
    bitWrite(pendingAction, pendingLeftMotor, false);
    if (bitRead(currentMove, toDoRotation) == true)
    {
      ComputeNewLocalization(0x01);
    }
    else {
      ComputeNewLocalization(0xff);
    }
    Serial.print("min level left");
    Serial.print(Wheels.GetMinLevel(leftWheelId));
    Serial.print("max:");
    Serial.println(Wheels.GetMaxLevel(leftWheelId));
    Serial.print("min level right");
    Serial.print(Wheels.GetMinLevel(rightWheelId));
    Serial.print("max:");
    Serial.println(Wheels.GetMaxLevel(rightWheelId));
  }
}

void ComputeNewLocalization(uint8_t param)  // compute localization according to the wheels rotation
{
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
        /*
        float deltaAlphaRadian = asin(deltaD / (2 * iRobotWidth));
        Serial.println(deltaAlphaRadian);
        float rayon =  deltaRight / (2 * sin(deltaAlphaRadian));
        Serial.println(rayon);
        float arcCenter = 2 * (rayon - iRobotWidth / 2) * sin(deltaAlphaRadian);
        Serial.println(arcCenter);
        deltaPosX = deltaPosX + arcCenter * cos(alphaRadian + deltaAlphaRadian);
        deltaPosY = deltaPosY + arcCenter * sin(alphaRadian + deltaAlphaRadian);
        alpha = (deltaAlphaRadian * 2 * 180 / PI + alpha);
        Serial.print("delta alpha:");
        Serial.print(deltaAlphaRadian * 2 * 180 / PI);
        //     Serial.print(" deltaC:");
        //    Serial.print(deltaC);
        Serial.print(" ");
        */
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
  if ( param == 0x01)
  {
    deltaAlpha = ((deltaRight + deltaLeft) * 180 / (PI * iRobotWidth )); //
    if (bitRead(currentMove, toDoClockwise) == false )
    {
      deltaAlpha = -deltaAlpha;
    }
    alpha = deltaAlpha + alpha;
#if defined(debugLocalizationOn)
    Serial.print("new pox X:");
    Serial.print(posX);
    Serial.print(" Y:");
    Serial.print(posY);
    Serial.print(" delta alpha:");
    Serial.print(deltaAlpha);
    Serial.print(" alpha:");
    Serial.println(alpha);
#endif
  }
  if (param == 0xff )
  {
    if (bitRead(currentMove, toDoStraight) == true)
    {
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
    }
#if defined(debugLocalizationOn)
    Serial.print("new pox X:");
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


void EchoServoAlign()    // to align echo servo motor with robot
{
  myservo.attach(servoPin);
  myservo.write(pulseValue[(nbPulse + 1) / 2]);  // select the middle of the pulse range
  delay(1000);
  myservo.detach();
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

void SendStatus()
{

  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x65; //
  PendingDataReqSerial[1] = appStat; //
  PendingDataReqSerial[2] = actStat;
  PendingDataReqSerial[3] = diagPower;
  PendingDataReqSerial[4] = 0x00; // no more than 3 conscutives non asccii bytes
  PendingDataReqSerial[5] = diagMotor;
  PendingDataReqSerial[6] = diagConnection;
  PendingDataReqSerial[7] = diagRobot;
  if (posX >= 0)
  {
    PendingDataReqSerial[8] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[8] = 0x2d;
  }
  PendingDataReqSerial[9] = uint8_t(abs(posX) / 256);
  PendingDataReqSerial[10] = uint8_t(abs(posX));
  if (posY >= 0)
  {
    PendingDataReqSerial[11] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[11] = 0x2d;
  }
  PendingDataReqSerial[12] = uint8_t(abs(posY) / 256);
  PendingDataReqSerial[13] = uint8_t(abs(posY));

  if (alpha >= 0)
  {
    PendingDataReqSerial[14] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[14] = 0x2d;
  }
  int angle = alpha;
  PendingDataReqSerial[15] = uint8_t(abs(angle) / 256);
  PendingDataReqSerial[16] = uint8_t(abs(angle));
  PendingDataReqSerial[17] = 0x00;
  int  northOrientation = saveNorthOrientation;
  if (toDo == 0x00 && (actStat != 0x66 && actStat != 0x68) && millis() - delayAfterStopMotors > 500 )
  {
    northOrientation = NorthOrientation();
  }
  PendingDataReqSerial[18] = uint8_t(northOrientation / 256);
  PendingDataReqSerial[19] = uint8_t(northOrientation);
  PendingDataReqSerial[20] = 0x00;
  PendingDataReqSerial[21] = currentLocProb;
  PendingDataLenSerial = 0x16; // 6 longueur mini max 25 pour la gateway
}
void SendPowerValue()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  PendingDataReqSerial[3] = 0x00;
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = 0x00;
  PendingDataLenSerial = 0x06; // 6 longueur mini max 25  pour la gateway
}
void StopAll()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  myservo.detach();
  StopEchoInterrupt(true, true);
  Wheels.StopWheelControl(true, true, 0, 0);
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  appStat = 0xff;       // update robot status
  actStat = 0x00;       // clear action status
  toDo = 0x00;           // cleat toDo byte
  currentMove = 0x00;    // clear current move
  saveCurrentMove = 0x00; // clear saveCurrent move
  pendingAckSerial = 0x00; // clear pending acknowledgement
  SendStatus();
}
void StartEchoInterrupt(boolean frontOn, boolean backOn)
{
#if defined(debugObstacleOn)
  Serial.print("start echo interrupt ");
  if (frontOn == true)
  {
    Serial.print("front");
  }
  if (backOn == true)
  {
    Serial.print( " back");
  }
  Serial.println();
  scanNumber = 0;
#endif
  //  digitalWrite(echoPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(echoPinInterrupt), obstacleInterrupt, RISING);
  echo.StartDetection(frontOn, backOn, echo3, echo4, echoCycleDuration);
  echo.SetAlertOn(frontOn, (frontLenght + securityLenght), backOn, (backLenght + securityLenght), echo3Alert, 0, echo4Alert, 0);
}
void StopEchoInterrupt(boolean frontOff, boolean backOff)
{
#if defined(debugObstacleOn)
  Serial.println("stop echo interrupt");
#endif
  echo.StopDetection(!frontOff, !backOff, echo3, echo4);
  detachInterrupt(digitalPinToInterrupt(echoPinInterrupt));
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
      timeBetweenOnOffObstacle = millis();     // use to compute delay between obstacle detection sitch on off
    }
    else
    {
      if (millis() - timeBetweenOnOffObstacle > delayBeforeRestartAfterAbstacle)
      {
        bitWrite(waitFlag, toEndPause, true);     // position bit pause to end
        digitalWrite(echoPinInterrupt, LOW);      // to reactivate echo interrupt
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
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  bitWrite(diagRobot, 0, 1);       // position bit diagRobot
  Serial.println("stop obstacle");
  //  delayToStopWheel = millis();
  Horn(true, 750);
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
}

void ResumeMove()  // no more obstacle resume moving
{
  bitWrite(diagRobot, 0, 0);       // position bit diagRobot
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
  if (abs(lenToDo) >= minDistToBeDone || rotation >= minRotToBeDone)
  {
    pendingStraight = lenToDo;
    Move(rotation, lenToDo);
  }
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
  if (reboot == true && millis() > rebootDuration)  // end of arduino reboot
  {
    diagRobot = 0x00;
    reboot = false;
    waitFlag = 0x00;
    appStat = 0x00;
    Horn(true, 250);
  }
}

void obstacleInterrupt()        // Obstacles detection system set a softare interrupt due to threshold reaching
{
  Serial.print("obstacle:");
  Serial.println(echo.GetDistance(echo.GetAlertEchoNumber()));
  bitWrite(waitFlag, toPause, true);     // position bit pause to end
  //  digitalWrite(echoPinInterrupt, LOW);
  PauseMove();
}

void WheelInterrupt()   // wheel controler set a software interruption due to threshold reaching
{
  uint8_t wheelId = Wheels.GetLastWheelInterruptId();  // get which wheel Id reached the threshold
  if (wheelId != 5)
  {
    Wheels.ClearThershold(wheelId);                      // clear the threshold flag to avoid more interruption
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

    if (wheelId == rightWheelId)
    {
      rightMotor.StopMotor();                         // stop firstly the motor that reached the threshold
      leftMotor.StopMotor();                          // stop the other motor to avoid to turn round
    }
    StopEchoInterrupt(true, true);                    // stop obstacles detection
    delay(200);                                       // wait a little for robot intertia
    Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
    detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
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
    if ( bitRead(currentMove, toDoStraight) == true)      // robot was moving straight
    {
      ComputeNewLocalization(0xff);                       // compute new  robot position
      bitWrite(currentMove, toDoStraight, false) ;        // clear flag todo straight
    }
    if ( bitRead(currentMove, toDoRotation) == true)      // robot was turning around
    {
      ComputeNewLocalization(0x01);                       // compute new  robot position
      bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation
    }
    bitWrite(pendingAction, pendingLeftMotor, false);     // clear the flag pending action motor
    bitWrite(pendingAction, pendingRightMotor, false);    // clear the flag pending action motor
    if (bitRead(toDo, toDoStraight) == false && bitRead(waitFlag, toWait) == 0 && actStat == moving)           // no more move to do
    {
      actStat = moveEnded;                                      // status move completed
      toDo = 0x00;                                         // clear flag todo
    }
    SendStatus();
  }
  else                      // wheel mode pulse
  {
    leftMotor.StopMotor();                             // stop firstly the motor that reached the threshold
    rightMotor.StopMotor();                            // stop the other motor to avoid to turn round
  }
}

void CalibrateWheels()           // calibrate encoder levels and leftToRightDynamicAdjustRatio according to mesurments
{
  saveLeftWheelInterrupt = 0;
  saveRightWheelInterrupt = 0;
  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);
  Wheels.StartWheelControl(true, false, 0 , true, false, 0 , false, false, 0, false, false, 0);
  leftMotor.RunMotor(0,  leftMotorPWM );
  rightMotor.RunMotor(1, rightMotorPWM * leftToRightDynamicAdjustRatio);
  delay(3000);                                    // run motors 3 seconds
  rightMotor.StopMotor();                         // stop firstly the motor that reached the threshold
  leftMotor.StopMotor();                          // stop the other motor to avoid to turn round
  StopEchoInterrupt(true, true);                    // stop obstacles detection
  delay(200);                                       // wait a little for robot intertia
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
  leftWheeelCumulative = leftWheeelCumulative + Wheels.GetCurrentHolesCount(leftWheelId);
  rightWheeelCumulative = rightWheeelCumulative + Wheels.GetCurrentHolesCount(rightWheelId);
  unsigned int minLeftLevel = Wheels.GetMinLevel(leftWheelId);
  unsigned int maxLeftLevel = Wheels.GetMaxLevel(leftWheelId);
  unsigned int minRightLevel = Wheels.GetMinLevel(rightWheelId);
  unsigned int maxRightLevel = Wheels.GetMaxLevel(rightWheelId);
  unsigned int leftAvgLevel = (maxLeftLevel + minLeftLevel) / 2;
  unsigned int leftDiffLevel = (maxLeftLevel - minLeftLevel) / 3;
  unsigned int leftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int rightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  leftIncoderLowValue = leftAvgLevel - leftDiffLevel;
  leftIncoderHighValue = leftAvgLevel + leftDiffLevel;
  unsigned int rightAvgLevel = (maxRightLevel + minRightLevel) / 2;
  unsigned int rightDiffLevel = (maxRightLevel - minRightLevel) / 3;
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
  compass.read();
  float northOrientation = compass.heading();
#if defined(debugLocalizationOn)
  Serial.print(" magneto orientation: ");
  Serial.println(northOrientation);
#endif
  saveNorthOrientation = int(northOrientation);
  return saveNorthOrientation;
}

#define minAlign 2
#define maxAlign 358
void northAlign(unsigned int northAlignShift)
{ // aligning anti-clockwise positive    - robot rotation clockwise positive
  uint8_t retry = 0;
  boolean aligned = false;
  actStat = aligning; // align required
  while (aligned == false && retry <= 15)
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
      toDo = 0x00;
      bitWrite(toDo, toDoAlign, 1);       // position bit toDo
      appStat = appStat & 0x1f;
      if (alignTarget <= 180)
      {
        if (alignTarget > 10)
        {
          Rotate(alignTarget);
        }
        else
        {
          bLeftClockwise = true;
          bRightClockwise = bLeftClockwise;
          //        bitWrite(currentMove, toDoClockwise, true);
          //        bitWrite(saveCurrentMove, toDoClockwise, true);
          pulseMotors(2);
        }
      }
      else
      {
        if (alignTarget < 350)
        {
          Rotate(alignTarget - 360);
        }
        else
        {
          bLeftClockwise = false;
          bRightClockwise = bLeftClockwise;
          //        bitWrite(currentMove, toDoClockwise, false);
          //        bitWrite(saveCurrentMove, toDoClockwise, false);
          pulseMotors(2);
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
        bitWrite(toDo, toDoAlign, 0);       // clear bit toDo
        SendEndAlign();
      }
    }
  }
}
void SendEndAlign()
{

  int northOrientation = NorthOrientation();
  actStat = alignEnded; // " align ended
  PendingDataReqSerial[0] = actStat;
  PendingDataReqSerial[1] = appStat;
  PendingDataReqSerial[3] = 0x00;
  PendingDataReqSerial[4] = uint8_t(northOrientation / 256);
  PendingDataReqSerial[5] = uint8_t(northOrientation);
  PendingDataReqSerial[6] = 0x00;
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = 0x00;
  PendingDataReqSerial[9] = 0x00;
  PendingDataLenSerial = 0x0a;

  PendingReqSerial = PendingReqRefSerial;
  myservo.write(pulseValue[(nbPulse + 1) / 2]); // remise au centre
  delay(1000);
  // myservo.detach();  // attaches the servo on pin 4 to the servo object
  // digitalWrite(power1Pin, LOW);
  numStep = 0;

}













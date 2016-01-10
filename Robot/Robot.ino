String Version = "RobotV0";
// uncomment #define debug to get log on serial link
#define debugScanOn true
#define debugMoveOn true
#define debugObstacleOn true
#define debugLocalizationOn true
#define debugMotorsOn true
#define debugWheelControlOn true
#define debugConnection true
#include <Servo.h>  // the servo library
#include <math.h>
#include <EEPROM.h>  // 
#include <Stepper.h>
#include <SoftwareSerial.h>
//ServoTimer2 myservo;  // create servo object to control a servo
Servo myservo;  // create servo object to control a servo

//-- comunication --
#include <SerialNetworkVariable.h>   // needed for RF434 network
#include <SerialNetworkVoid.h>  // needed for RF434 network
#include <motorControl2342L.h>  // library for motors control
uint8_t PendingReqRef = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRef = 0xbf; // pending request (if 0x00 none)- copy fo retry
uint8_t PendingReqRefSerial = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRefSerial = 0xbf; // pending request (if 0x00 none)- copy fo retry
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
uint8_t trameNumber = 0;     // frame number to send
uint8_t lastAckTrameNumber = 0;  // last frame number acknowledged by the server
//uint8_t pendingAck = 0;
uint8_t pendingAckSerial = 0;    // flag waiting for acknowledged
int retryCount = 0;            // number of retry for sending

//-- General Robot parameters --
boolean reboot = true;
int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iRightMotorMaxrpm = iLeftMotorMaxrpm ; // (Value unknown so far - Maximum revolutions per minute for left motor)
float fMaxrpmAdjustment;  // will be used to compensate speed difference betweeen motors
int iLeftWheelDiameter = 62; //(in mm - used to measure robot moves)  65mn a vid
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
float iLeftMotorDemultiplierPercent = 100; // *12/9(1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel) used to fit speed encoder rotation to wheel rotation (motor 12 pinion teeth wheel 9 pinion teeth)
float iRightMotorDemultiplierPercent = iLeftMotorDemultiplierPercent; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
unsigned int iLeftTractionDistPerRev =  (PI * iLeftWheelDiameter) * 100 / (iLeftMotorDemultiplierPercent);
unsigned int iRightTractionDistPerRev = (PI * iRightWheelDiameter) * 100 / (iRightMotorDemultiplierPercent);
int iRobotWidth = 455; // distance beetwen the 2 wheels mm
float coeffGlissementRotation = 1.;
#define frontLenght  35 // from echo system
#define backLenght  12 // from echo system
#define securityLenght 50 // minimal obstacle distance 
#define rebootDuration 10000 // delay to completly start arduino

//-- power control --
int uRefMotorVoltage = 1200; // mVolt for maxRPM
#define power1Pin 53  // power 1 servomoteur
#define power1Value A15  // 12v power for motors
#define power2Value A14  // 9v power for arduino mega
#define power3Value A13  // 5V power for esp8266 and components
#define power1LowLimit 750   // minimum centi volt before warning
#define power2LowLimit 450   // minimum centi volt before warning
#define power3LowLimit 750  // minimum centi volt before warning
int power1Mesurt = 0;   // current power1 value
int power2Mesurt = 0;   // current power2 value
int power3Mesurt = 0;   // current power3 value

//-- left Motor connection --
#define leftMotorENA 6 // Arduino pin must be PWM
#define leftMotorIN1 26 // arduino pin for rotation control
#define leftMotorIN2 27  // arduino pin for rotation control
//-- right Motor connection --
#define rightMotorENB 3 // Arduino pin must be PWM
#define rightMotorIN3 25 // arduino pin for rotation control
#define rightMotorIN4 24  // arduino pin for rotation control

//-- motors control --
#define iLeftSlowPMW 150        // PMW value to slowdown motor at the end of the run
#define pendingLeftMotor 0      // define pendingAction bit used for left motor
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPMW); // define left Motor
#define iRightSlowPMW iLeftSlowPMW   // PMW value to slowdown motor at the end of the run
#define pendingRightMotor 1    // define pendingAction bit used for right motor
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPMW); // define right Motor

//-- wheel control --
#define wheelSpeedRightPin 18  // arduino pin connected to IR receiver
#define wheelSpeedLeftPin 19   // arduino pin connected to IR receiver
#define leftWheelEncoderHoles 8  // number of holes of the encoder wheel
#define rightWheelEncoderHoles leftWheelEncoderHoles // number of holes of the encoder wheel
#define minInterruptDuration 35 // 35 used to avoid entering more than one time in the interrupt void
float avgRightWheelSpeed = 0;    // average right wheel speed
float avgLeftWheelSpeed = 0;    // average left wheel speed
int iLeftRevSpeed;              // instant left wheel speed
int iRightRevSpeed;             // instant right wheel speed
#define sizeOfLeftRev 8 // size of the array containing latest revolution wheel speed
#define sizeOfRightRev 8 // size of the array containing latest revolution wheel speed
unsigned int instantLeftWheelRevSpeed[sizeOfLeftRev];  // array containing latest revolution left wheel speed
unsigned int instantRightWheelRevSpeed[sizeOfRightRev]; // array containing latest revolution right wheel speed
uint8_t leftWheelSpeedCount = 0x00;                    // pointer to instantLeftWheelRevSpeed position
uint8_t rightWheelSpeedCount = 0x00;                   // pointer to instantRightWheelRevSpeed position
unsigned long prevLeftWheelInterrupt = 0;              // copy of previous left wheel interrupt count
volatile unsigned long leftWheelInterrupt = 0;         // number of interrupts for the left encoder (uptated by interrupt >> volatile)
volatile unsigned long leftTimeInterrupt = 0;          // last time left encoder interrupt taken into account (uptated by interrupt >> volatile)
unsigned long prevRightWheelInterrupt = 0;              // copy of previous right wheel interrupt count
volatile unsigned long rightWheelInterrupt = 0;        // number of interrupts for the right encoder (uptated by interrupt >> volatile)
volatile unsigned long rightTimeInterrupt = 0;         // last time right encoder interrupt taken into account (uptated by interrupt >> volatile)
unsigned long saveLeftWheelInterrupt = 0;             // copy of previous number of interrupts for the left encoder
unsigned long saveRightWheelInterrupt = 0;            // copy of previous number of interrupts for the right encoder
int iSpeed = 408;                                     // default expected robot speed mm/s (408 maxi)


//-- move control --
# define bForward  true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
unsigned long iLeftCentiRevolutions;  // nmuber of done revolutions * 100
unsigned long iRightCentiRevolutions; // nmuber of done revolutions * 100
#define toDoMove 1      // something  to do
#define toDoRotation 2  // rotation to do
#define toDoStraight 3  // straight move to do
#define toDoBackward 4  // straight move to do is backward
#define toDoClockwise 5 // rotate clockwise
#define toDoWait 1    // wait before moving
#define toDoEndPause 2     // could restart
#define toDoPause 3     // move to be temporaly paused
//unsigned long iDistance = 0; // mm
int pendingStraight = 0;   // copy of straight distance to be done
int reqAng = 0;          // requested rotation
int reqMove = 0;         // requested move
uint8_t resumeCount = 0; // nb of echo check before resume move
int movePingInit;     // echo value mesured before moving

//-- scan control --
#define servoPin 9    // stepper
#define toDoScan 0    // toDo bit for scan request
#define echoFront 20 //24  // arduino pin for mesuring echo delay for front 
#define trigFront  22 //25     // arduino pin for trigerring front echo
#define echoBack  21 //22  // arduino pin for mesuring echo delay for back 
#define trigBack  23    // arduino pin for trigerring back echo
#define nbPulse 15    // nb of servo positions for a 360° scan
int numStep = 0;     // current scan step number
int nbSteps = 0;     // number of steps to be done for a scan
boolean switchFB = 0;  // switch front back scan
//float coefAngRef;
#define nbPulse 15    // nb of servo positions for a 360° scan
int pulseValue[nbPulse] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165}; // corresponding to 360° split in 15 steps
float coefAngRef = PI / (pulseValue[14] - pulseValue[0]);  // angle value beetwen 2 pulses
uint8_t pulseNumber = 0;  // pointer to pulseValue array
int distFSav = 0;     // saved front echo distance in cm
int distBSav = 0;      // saved back echo distance in cm
int valAng;  // servo orientation
float AngleRadian;  // echo servo orientation in radian
float AngleDegre;    // echo servo orientation in degre
int scanOrientation = 0;
volatile unsigned long prevMicros = 0; // used during move to dynamicaly avoid obstacles
volatile unsigned int lastMicro = 0; // used during move to dynamicaly avoid obstacles
uint8_t echoCurrent = 0; // used during move to determine which front or back is used
uint8_t trigCurrent = 0; // used during move to determine which front or back is used
boolean trigOn = false;  // flag for asynchronous echo to know if trigger has been activated
unsigned int scanNumber = 0;   // count number of scan retry for asynchronous echo

//-- timers --
unsigned long timeAppli; // cycle applicatif
unsigned long timeStatut;  // cycle envoi du statut au master
unsigned long timeNetwStat; // cycle affichage des stat RF433 sur serial
unsigned long timeRTC;  // cycle demande heure
unsigned long timeScanFront;  // cycle demande heure
unsigned long timeScanBack;  // cycle demande heure
//unsigned long timeSendSec;  //
unsigned long timeSendSecSerial;  // used to check for acknowledgment of secured frames
unsigned long timeReceiveSerial;  // used to regularly check for received message
unsigned long timePower1Check;  // used to regularly check for power values
unsigned long delayToStopWheel = 0; // used to postpone a little bit the wheels control after stopping motors
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
#define delayBetween2Scan 1500  // delay between to scan steps - must take into account the transmission duration
#define delayBetweenScanFB 700  // delay between front and back scan of the same step
#define delayBetweenInfo 5000   // delay before sending new status to the server  
#define delayPowerCheck 5000    // delay before next checking of power

//-- robot status & diagnostics
uint8_t diagMotor = 0x00;   // bit 0 pb left motor, 1 right moto, 2 synchro motor
uint8_t diagPower = 0x00;   // bit 0 power1, 1 power2, 2 power3 0 Ok 1 Ko
uint8_t diagConnection = 0x01;   // bit 0 pending serial link
uint8_t diagRobot = 0xff;     // to be cleared after complete reboot
uint8_t toDo = 0x00;          // flag kind of move to do
uint8_t waitFlag = 0x00;     // used to pause and waiting before starting

uint8_t currentMove = 0x00;   // flag kind of current move
uint8_t pendingAction = 0x00; // flag pending action to be started
uint8_t sendInfoSwitch = 0x00;
uint8_t saveCurrentMove = 0x00;   // flag last kind of current move
boolean moveForward;           // flag to indicate if requested move is forward or backward
uint8_t AppStat = 0xff; // statut code application 00 active ff stopped 1er demi octet mouvement 2eme demi scan
uint8_t actStat = 0xff; // staut action en cours

//-- space localizarion --
float alpha = 0; // current orientation
long posX = 0;   // current x position
long posY = 0;   // current y position
float targetAlpha = 0; // orientation target position after moving
long targetX = 0;  // x target position after moving
long targetY = 0;// x target position after moving
long deltaPosX = 0;  // incremental x move
long deltaPosY = 0;  // incremental y move


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
  Serial.begin(38400);            // for debug
  Serial2.begin(SpeedNetwSerial); // to communicate with the server through a serial Udp gateway
  Serial.println(Version);
  int addrEeprom = 0;
  byte valueEeprom;
  valueEeprom = EEPROM.read(addrEeprom);  // id of the arduino stored in eeprom
  Serial.print("ArduinoID=");
  Serial.println(valueEeprom, DEC);
  // define gpio mode
  pinMode(trigFront, OUTPUT);
  digitalWrite(trigBack, LOW);
  pinMode(trigBack, OUTPUT);
  digitalWrite(trigFront, LOW);
  pinMode(echoFront, INPUT);
  pinMode(echoBack, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(power1Pin, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(wheelSpeedLeftPin, INPUT);
  pinMode(wheelSpeedRightPin, INPUT);
  // set initial LED
  digitalWrite(power1Pin, LOW);
  digitalWrite(blueLed, HIGH);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, HIGH);
  //
  //  coefAngRef = PI / (pulseValue[14] - pulseValue[0]);
  myservo.attach(servoPin);                       // attach the echo servo
  myservo.write(pulseValue[(nbPulse + 1) / 2]);  // set echo servo at the middle position
  delay(1000); // time fot the servo to reach the right position
  myservo.detach();
}

void loop() {
  CheckEndOfReboot();
  if (millis() - delayCheckSpeed > 250 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  { // Compute left and right wheels average speed over past seconds (sizeOfLeftRev*250 seconds). Updated every 250 milliseconds.
    ComputeAverageWheelsSpeed();
  }
  if (millis() - delayCheckPosition > 200 && (prevLeftWheelInterrupt != leftWheelInterrupt || prevRightWheelInterrupt != rightWheelInterrupt) && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  { // Compute new robot position
    delayCheckPosition = millis();
    //  CheckMoveSynchronisation();
    ComputeNewLocalization(0x00);
  }
#if defined(debugWheelContolOn)
  if (millis() - delayPrintSpeed > 1000 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  {
    PrintSpeed();
  }
#endif
  if (bitRead(waitFlag, toDoPause) == 1 && (millis() - checkObstacleTimer) > 2000 ) // robot in pause status
  {

    CheckObstacle();

    if ( bitRead(waitFlag, toDoEndPause) == 1 ) // no more obstacle
    {
      //       resumeCount=0;
      ResumeMove();
      bitWrite(waitFlag, toDoPause, 0);
      bitWrite(waitFlag, toDoEndPause, 0);
      bitWrite(waitFlag, toDoWait, 0);       //
    }

  }
  if ((millis() - checkObstacleTimer) > (durationMaxEcho / 1000) && trigOn == true && bitRead(waitFlag, toDoPause) != 1) // triger >> on get result
  {
    CheckObstacle();
    //    checkObstacleTimer = millis();
  }
  if (millis() - checkObstacleTimer > 750 && echoCurrent != 0 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  {
    CheckObstacle();
    //    checkObstacleTimer = millis();
  }
  if (saveCurrentMove != currentMove && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false ) // detect end of move
  {
    DetectEndOfMove();
  }

  if (bitRead(pendingAction, pendingLeftMotor) == true && bitRead(waitFlag, toDoWait) == 0)     //
  {
    int feedBackLeftMotor = leftMotor.CheckMotor( (avgLeftWheelSpeed * iLeftMotorDemultiplierPercent) / 100, ((leftWheelInterrupt * 100) / leftWheelEncoderHoles) * iLeftMotorDemultiplierPercent / 100 );
    switch (feedBackLeftMotor)
    {
      case -2:   // motor is stopped
        {
          leftMotor.StopMotor();
          rightMotor.StopMotor();
          bitWrite(diagMotor, 0, 1);       // position bit diagMotor
#if defined(debugMotorsOn)
          Serial.println("left motor pb");
#endif
          delayToStopWheel = millis();
          bitWrite(pendingAction, pendingLeftMotor, false);
          toDo = toDo & (~currentMove & B00001100); // remove current move bit from toDo
          break;
        }
      case -1:  // motor end of move
        {
          bitWrite(pendingAction, pendingLeftMotor, false);
          rightMotor.StopMotor();  // stop synchrone des 2 moteurs
          delayToStopWheel = millis();
          toDo = toDo & (~currentMove & B00001100); // remove current move bit from toDo

          break;
        }
      case 0:
        {
          bitWrite(pendingAction, pendingLeftMotor, false);
          toDo = toDo & (~currentMove & B00001100);  // remove current move bit from toDo
#if defined(debugMoveOn)
          Serial.print("todo Left 0x");
          Serial.print(toDo, HEX);
          Serial.print(" curr 0x");
          Serial.println(currentMove, HEX);
#endif
          break;
        }
      default:
        break;
    }

  }
  if (bitRead(pendingAction, pendingRightMotor) == true && bitRead(waitFlag, toDoWait) == 0)
  {

    //    Serial.println(avgRightWheelSpeed );
    int feedBackRightMotor = rightMotor.CheckMotor( (avgRightWheelSpeed * iRightMotorDemultiplierPercent) / 100  , ((rightWheelInterrupt * 100) / rightWheelEncoderHoles) * iRightMotorDemultiplierPercent / 100 );
    switch (feedBackRightMotor)
    {
      case -2:
        rightMotor.StopMotor();
        leftMotor.StopMotor();
        bitWrite(diagMotor, 1, 1);       // position bit diagMotor
#if defined(debugMotorsOn)
        Serial.println("right motor pb");
#endif
        delayToStopWheel = millis();
        bitWrite(pendingAction, pendingRightMotor, false);
        toDo = toDo & (~currentMove & B00001100); // remove current move bit from toDo
        break;
      case -1:
        bitWrite(pendingAction, pendingRightMotor, false);
        leftMotor.StopMotor();  // stop synchrone des 2 moteurs
        delayToStopWheel = millis();
        toDo = toDo & (~currentMove & B00001100); // remove current move bit from toDo
        break;
      case 0:
        {
          bitWrite(pendingAction, pendingRightMotor, false);
          toDo = toDo & (~currentMove & B00001100); // remove current move bit from toDo
#if defined(debugMoveOn)
          Serial.print("todo Right ox");
          Serial.print(toDo, HEX);
          Serial.print(" curr 0x");
          Serial.println(currentMove, HEX);
#endif
          break;
        }
      default:
        break;
    }


  }
  if (bitRead(toDo, toDoStraight) == true && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false)
  {
    if ( bitRead(currentMove, toDoRotation) == true )  // end of first move step (rotation) move forward to be done
    {
#if defined(debugMoveOn)
      Serial.println("end rotation still straight move todo");
#endif
      delayToStopWheel = 0; // to avoid pending wheel stopping
      ComputeNewLocalization(0x01);
    }
    MoveForward(pendingStraight) ;
  }
  //  if (delayToStopWheel != 0 && (millis() - delayToStopWheel) > 80 && toDo == 0x00 ) // delay to completely stop robot
  if (delayToStopWheel != 0 && (millis() - delayToStopWheel) > 80  ) // delay to completely stop robot
  {
    stopWheelControl();
    delayToStopWheel = 0;
    SendStatus();
  }

  if ((millis() - timePower1Check) >= delayPowerCheck ) // regularly check power values
  {
    PowerCheck();

  }

  if (bitRead(toDo, toDoScan) == 1)          // check if for scan request
  {
    ScanPosition();
  }


  if (bitRead(toDo, toDoMove) == 1) {         // check if for move request
    //  Serial.println("move");
    Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
    AppStat = AppStat | 0xf0;
  }

  // ***  keep in touch with the server
  int getSerial = Serial_have_message();  // check if we have received a message
  if (getSerial > 0)                      // we got a message
  {

    TraitInput(DataInSerial[2]);          // analyze the message
  }
  if (PendingReqSerial != 0x00 )           // check if we have a message to send
  {
    DataToSendSerial();                    // send message on the serial link
    timeSendSecSerial = millis();          // reset timer
  }
  if (retryCount >= 3)
  {
    pendingAckSerial = 0x00;
    retryCount = 0;
  }
  if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 3) {
    ReSendSecuSerial();
#if defined(debugConnection)
    Serial.println("retry");
#endif
    timeSendSecSerial = millis();
    retryCount = retryCount + 1;
  }
  if (millis() - timeReceiveSerial >= 30000)
  {
    //   Serial.println(timeReceiveSerial);
    bitWrite(diagConnection, 1, 0);       // established connection
  }

  if (millis() - timeSendInfo >= delayBetweenInfo && toDo == 0x00)  // alternatively send status and power to the server
  {
    if (sendInfoSwitch % 2 == 0)
    {
      SendStatus();                 // send robot status to the server
    }
    if (sendInfoSwitch % 2 == 1)
    {
      SendPowerValue();             // send power info to the server
    }
    sendInfoSwitch = sendInfoSwitch + 1;
    timeSendInfo = millis();
  }
  // *** refresh LED
  if (millis() - timeAffLed > 1000)
  {
    AffLed();
    timeAffLed = millis();
  }
}

void TraitInput(uint8_t cmdInput) {     // wet got data on serial
  //  Serial.println("serialInput");
  bitWrite(diagConnection, 0, 0);       // connection is active
  timeReceiveSerial = millis();         // reset check receive timer
  switch (cmdInput) {                   // first byte of input is the command type
    case 0x73: // commande s means stop
      Serial.println("cmd stop");
      AppStat = 0xff;
      leftMotor.StopMotor();
      rightMotor.StopMotor();
      toDo = 0x00;
      break;
    case 0x78: // commande x menas start
      Serial.println("cmd start");
      AppStat = 0x00;
      break;
    case 0x65: // commande e server request for robot status
      SendStatus();                                     // send robot status to server
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
#if defined(debugLocalizationOn)
      Serial.print("init X Y Alpha:");
      Serial.print(posX);
      Serial.print(" ");
      Serial.print(posY);
      Serial.print(" ");
      Serial.println(alpha);
#endif
      break;
    case 0x2b: // commande + means scan 360
      AppStat = AppStat & 0xf1;
      Serial.println("Scan");
      pulseNumber = 0;
      InitScan(nbPulse, 0);
      actStat = 0x66;
      bitWrite(toDo, toDoScan, 1);       // position bit toDo scan
      //      SendRFNoSecured();
      break;
    case 0x6d: // commande m means move
      AppStat = AppStat & 0x1f;
      actStat = 0x68; // moving
      Serial.print("Move: ");
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
      Serial.print(reqAng);
      reqMove = DataInSerial[7] * 256 + DataInSerial[8];
      if (DataInSerial[6] == 0x2d) {
        reqMove = -reqMove;
      }
      if (reqMove > 0)
      {
        bitWrite(toDo, toDoStraight, true);
        bitWrite(toDo, toDoBackward, false);   // to go forward
        //       bitWrite(currentMove, toDoClockwise, true);
        //        bitWrite(saveCurrentMove, toDoClockwise, true);
      }
      if (reqMove < 0)
      {
        bitWrite(toDo, toDoStraight, true);
        bitWrite(toDo, toDoBackward, true);  // to go backward
        //       bitWrite(currentMove, toDoClockwise, false);
        //       bitWrite(saveCurrentMove, toDoClockwise, false);
      }
#if defined(debugMoveOn)
      Serial.print(" ");
      Serial.println(reqMove);
      Serial.print("todo 0x");
      Serial.println(toDo, HEX);
#endif
      //   Serial.println(inc);
      break;
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
        //        PendingDataReq[0] = 0x00;
        //       Serial.println("noPending");
      }

      //Serial.println(inc);
      break;
    default:

      Serial.print("commande recue: ");
      Serial.println(cmdInput, HEX);

  }
}













void IncServo(int sens) {  // incremente ou decremente d'un pas le servo-moteur
  valAng = IncPulse(sens );
  myservo.write(valAng);

  delay(1000);
}
int IncPulse(int sens) { // calcul du pulse

  pulseNumber = pulseNumber + sens;
  return (pulseValue[(pulseNumber) % nbPulse]) ;

}
int PingFront() {
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
  /* for (int i=0;i>=sizeof(pulseValue);i++)
   {
   */
  //    PosServo();
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

  //    cm = lecture_echo / 58 * 1.25 + 7; // 1.25 coeff correction mesuree en reel +8 ecrat VS roues
  // cm = lecture_echo / 58;   // 1.25 coeff correction mesuree en reel +8 ecrat VS roues
  cm = deltaT / 58  ;
  return (cm);

  //  }
}
void InitScan(int nbS, int startingOrientation) {
  numStep = 0;
  nbSteps = nbS;
  scanOrientation = startingOrientation;
  digitalWrite(power1Pin, HIGH);
  delay(500);
  myservo.attach(servoPin);  // attaches the servo on pin 4 to the servo object
  myservo.write(pulseValue[scanOrientation]);
  delay(1000);

}

void ScanPosition() {

  // if ((millis() - timeScanFront) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 )
  if ((millis() - timeScanBack) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 && pendingAckSerial == 0x00)
  {

    timeScanFront = millis();
    switchFB = 1;
    distFSav = ScanOnceFront(numStep);
    //      vw_rx_start();        // dÃƒÂ©marrer la rÃƒÂ©ception
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


  actStat = 0x67; //" scan completed
  PendingDataReqSerial[0] = actStat;
  PendingDataReqSerial[1] = AppStat;
  PendingDataReqSerial[3] = 0x00;
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = 0x00;
  PendingDataReqSerial[6] = 0x00;
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = 0x00;
  PendingDataReqSerial[9] = 0x00;
  PendingDataLenSerial = 0x0a;

  PendingReqSerial = PendingReqRefSerial;
  myservo.write(pulseValue[(nbPulse + 1) / 2]); // remise au centre
  delay(1000);
  // myservo.detach();  // attaches the servo on pin 4 to the servo object
  digitalWrite(power1Pin, LOW);
  numStep = 0;

}
void Move(int orientation, int lenghtToDo)
{

  ComputeTargetLocalization(orientation, lenghtToDo);
  bitWrite(toDo, toDoMove, false);       // position bit toDo move
  if (orientation != 0)
  {
    //    bitWrite(toDo, toDoRotation, true);       // position bit toDo move
    Rotate(orientation);
  }

  if (lenghtToDo != 0)
  {
    //   bitWrite(toDo, toDoStraight, true);       // position bit toDo move
    pendingStraight = lenghtToDo;
  }
  /*
  if (lenghtToDo < 0)
  {
    //   bitWrite(toDo, toDoBackward, true);       // position bit
  }
  else
  {
    //    bitWrite(toDo, toDoBackward, false);       // position bit
  }
  */
}
void Rotate( int orientation) {

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
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, iSpeed, lentghRightToDo, iSpeed);
    bitWrite(currentMove, toDoRotation, true);
    bitWrite(saveCurrentMove, toDoRotation, true);
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
  //
  Serial.println("rotate ");

}



// SendRFNoSecured();

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
  SendStatus();
  // move Toward
  if (lengthToDo != 0 )
  {
    Serial.print("moveToDo:");
    Serial.println(lengthToDo);
    lentghLeftToDo = abs(lengthToDo);
    lentghRightToDo = abs(lengthToDo);
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, iSpeed, lentghRightToDo, iSpeed);


    if (lengthToDo < 0)
    {
      bLeftClockwise = bForward;
      bRightClockwise = !bForward;
      moveForward = false;
      movePingInit = -PingBack();
      echoCurrent = echoBack;
      trigCurrent = trigBack;
      StartEchoInterrupt();
    }
    else
    {
      bLeftClockwise = !bForward;
      bRightClockwise = bForward;
      moveForward = true;
      movePingInit = PingFront();
      echoCurrent = echoFront;
      trigCurrent = trigFront;
      StartEchoInterrupt();
    }
    Serial.print("dist obs:");
    Serial.println(movePingInit);
    Serial.println(iLeftCentiRevolutions );
    startMotors();
  }

  // Serial.println("move ok");

  //  actStat = 0x69; //" move completed
}




unsigned int  RotationCalculation(int orientation) {
  unsigned int distToDo = ((iRobotWidth * PI / 360) * orientation);
  Serial.print("distRot:");
  Serial.println(distToDo);
  return (distToDo);
}
/*
bool CheckFeedback()
{
  Serial.print("Checkfeedback:");
  Serial.print(abs(pulseValue[(pulseNumber) % nbPulse] ));
  servoFeedbackValue = analogRead(servoFeedbackPin);
  Serial.print(" ");
  Serial.println(servoFeedbackValue);
  bool retFeed = true;
  float retCode = abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue);
  retCode = retCode / abs(pulseFeedback[(pulseNumber) % nbPulse]);
  //  Serial.println(retCode);
  if (retCode > 0.02)
  {
    delay(50);
    servoFeedbackValue = analogRead(servoFeedbackPin);
    retCode = abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue);
    retCode = retCode / abs(pulseFeedback[(pulseNumber) % nbPulse]);
    Serial.print(">");
    Serial.println(servoFeedbackValue);
    //    Serial.println(retCode);
  }
  if (retCode > 0.02)
  {
    Serial.print("pb feedback:");
    Serial.print(retCode);
    Serial.print(" ");
    Serial.println(servoFeedbackValue);

    retFeed = false;
  }
  return retFeed;
}
*/
void SendScanResultSerial (int distF, int distB)
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
  PendingDataLenSerial = 0x0d;
  pendingAckSerial = 0x01;
  PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  SendSecuSerial();
  timeSendSecSerial = millis();
}


void PowerCheck()
{
  timePower1Check = millis(); //
  //  Serial.print("power1:");
  power1Mesurt = 3 * map(analogRead(power1Value), 0, 1023, 0, 467);
  //  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 12v
  //  Serial.println("cVolt");
  //  Serial.print("power2:");
  if (power1Mesurt < 900)
  {
    bitWrite(diagPower, 0, 1);
  }
  else
  {
    bitWrite(diagPower, 0, 0);
  }
  power2Mesurt = 2 * map(analogRead(power2Value), 0, 1023, 0, 456);
  //  Serial.print(power2Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  //  Serial.println("cV");
  if (power2Mesurt < 450)
  {
    bitWrite(diagPower, 1, 1);
  }
  else
  {
    bitWrite(diagPower, 1, 0);
  }
  power3Mesurt = 2 * map(analogRead(power3Value), 0, 1023, 0, 540);
  if (power3Mesurt < 800)
  {
    bitWrite(diagPower, 2, 1);
  }
  else
  {
    bitWrite(diagPower, 2, 0);
  }
  if (diagPower >= 0x07)
  {
    //  StopAll();
  }
  /*
  Serial.print("power3:");
  Serial.print(power3Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.println("cV");
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  PendingDataReqSerial[3] = uint8_t(power3Mesurt / 10);
  PendingDataReqSerial[4] = uint8_t(diagPower);
  PendingDataReqSerial[5] = 0x00;
  PendingDataLenSerial = 0x06; // 6 longueur mini pour la gateway
  */

}

void startMotors()
{

  bitWrite(pendingAction, pendingLeftMotor, true);
  bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
  Serial.print("reqSpeed:");
  Serial.print(iLeftRevSpeed);
  Serial.print(" ");
  Serial.println(iRightRevSpeed);
#endif
  //  bitWrite(diagMotor, 0, 0);       // position bit diagMotor left motor
  //  bitWrite(diagMotor, 1, 0);       // position bit diagMotor right motor
  diagMotor = 0x00;
  StartLeftWheelSpeedControl();   //
  StartRightWheelSpeedControl();   //
  leftMotor.TurnMotor(bLeftClockwise, iLeftCentiRevolutions, iLeftRevSpeed);
  rightMotor.TurnMotor(bRightClockwise, iRightCentiRevolutions, iRightRevSpeed);

}

void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, int iLeftSpeed, unsigned long iRightDistance, int iRightSpeed)
{

  float powerAdustment = float (uRefMotorVoltage) / (3 * map(analogRead(power1Value), 0, 1023, 0, 467)); // speed adjustement to power supply
  iLeftCentiRevolutions = (iLeftDistance * 100 / iLeftTractionDistPerRev) ; // Centi-revolutions
  iRightCentiRevolutions = (iRightDistance * 100 / iRightTractionDistPerRev) ; // ms

  iLeftRevSpeed = (iLeftSpeed * 60 ) / iLeftTractionDistPerRev; // revolutions per minute
  iRightRevSpeed = (iRightSpeed * 60) / iRightTractionDistPerRev; // revolutions per minute
}

void StartLeftWheelSpeedControl()
{
#if defined(debugWheelControlOn)
  Serial.println("startlefttctrl");
#endif
  for (int i = 0; i < sizeOfLeftRev; i++)
  {
    instantLeftWheelRevSpeed[i] = 0 ; // init avec expected speed iLeftRevSpeed
  }
  avgLeftWheelSpeed = 0;

  if (digitalRead(wheelSpeedLeftPin) == true)
  {
    attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, FALLING);
  }

  leftWheelInterrupt = 0;
  saveLeftWheelInterrupt = 0;
  //  deltaPosX = 0;
  //  prevLeftWheelInterrupt=0;
}
void StopLeftWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin));
}
void LeftWheelCount() // count right encoder interrupt
{

  if (millis() - leftTimeInterrupt > minInterruptDuration) // check minimum time between 2 "interrupt"
  {
    leftWheelInterrupt++;
  }
  leftTimeInterrupt = millis();
}
void StartRightWheelSpeedControl()
{
#if defined(debugWheelControlOn)
  Serial.println("startRightctrl");
#endif
  for (int i = 0; i < sizeOfRightRev; i++)
  {
    instantRightWheelRevSpeed[i] = 0; // init avec expected speediRightRevSpeed
  }
  avgRightWheelSpeed = 0;

  if (digitalRead(wheelSpeedRightPin) == true)
  {
    attachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin), RightWheelCount, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin), RightWheelCount, FALLING);
  }

  rightWheelInterrupt = 0;
  saveRightWheelInterrupt = 0;
  // deltaPosY = 0;

}
void StopRightWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin));
}
void RightWheelCount()        // count right encoder interrupt
{
  if (millis() - rightTimeInterrupt > minInterruptDuration) // check minimum time between 2 "interrupt"
  {
    rightWheelInterrupt++;
  }
  rightTimeInterrupt = millis();
}
void SpeedRightWheel()       // compute right wheel speed according to encoder interrupt and store values in instantRightWheelRevSpeed table
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
void SpeedLeftWheel()             // compute left wheel speed according to encoder interrupt and store values in instantLeftWheelRevSpeed table
{
  unsigned int deltaTime = millis() - prevLeftWheelIntTime;
  prevLeftWheelIntTime = millis();
  unsigned int deltaEncoder = leftWheelInterrupt - prevLeftWheelInterrupt;
  prevLeftWheelInterrupt = leftWheelInterrupt;
  // Serial.print("left wheel rpm:");
  instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeOfLeftRev] = 60 * (float (deltaEncoder * 1000 / deltaTime) / leftWheelEncoderHoles);
  //  Serial.println(instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeof(instantLeftWheelRevSpeed) / 2]);
  //  Serial.println(leftWheelSpeedCount % sizeOfLeftRev);
  leftWheelSpeedCount++;
}

void CheckMoveSynchronisation()       // check that the 2 wheels are rotating at the same pace
{
  float deltaMove = abs(leftWheelInterrupt - rightWheelInterrupt);
  bitWrite(diagMotor, 3, 0);       // position bit diagMotor
  if ( deltaMove > 2 && deltaMove / leftWheelInterrupt > 0.05)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    bitWrite(diagMotor, 3, 1);       // position bit diagMotor
    Serial.print("move synchro pb:");
    Serial.print(deltaMove);
    Serial.print(" ");
    Serial.println(abs(deltaMove / leftWheelInterrupt));
    if (bitRead(currentMove, toDoRotation) == true)
    {
      ComputeNewLocalization(0x01);
    }
    else {
      ComputeNewLocalization(0xff);
    }

    toDo = 0x00;
  }
}
void ComputeNewLocalization(uint8_t param)  // compute localization according to the wheels rotation
{
  float deltaAlpha = 0; // modification of robot angles (in degres) since last call of the function
  float deltaLeft = ((float (leftWheelInterrupt) - saveLeftWheelInterrupt) * PI * iLeftWheelDiameter / leftWheelEncoderHoles); // number of centimeters done by left wheel since last call of the function
  float deltaRight = ((float (rightWheelInterrupt) - saveRightWheelInterrupt) * PI * iRightWheelDiameter / rightWheelEncoderHoles); // number of centimeters done by right wheel since last call of the function

  if (bitRead(currentMove, toDoStraight) == true )
  { // if robot is supposed to go straight.

    if (bLeftClockwise == true)
    {
      deltaLeft = -deltaLeft;
    }

    if (bRightClockwise == false)
    {
      deltaRight = -deltaRight;
    }


    if (((deltaLeft != 0 && deltaRight != 0) ) )
    {
#if defined(debugWheelControlOn)
      Serial.print("newInt:");
      Serial.print(leftWheelInterrupt);
      Serial.print(" ");
      Serial.print(saveLeftWheelInterrupt);
      Serial.print(" ");
      Serial.print(rightWheelInterrupt);
      Serial.print(" ");
      Serial.print(saveRightWheelInterrupt);
      Serial.println(" ");
#endif
      saveLeftWheelInterrupt = leftWheelInterrupt;
      saveRightWheelInterrupt = rightWheelInterrupt;
      float deltaC = (deltaLeft + deltaRight) / 2; // Move of center of robot (middle of the 2 wheels) in centimeters.
      //      float moveLeft = (deltaLeft * PI * iLeftWheelDiameter)/leftWheelEncoderHoles;
      //     float moveRight = (deltaRight * PI * iRightWheelDiameter)/rightWheelEncoderHoles;

      deltaAlpha = asin((deltaRight - deltaLeft) / (iRobotWidth )); // Compute the modification of the angle since last call of the function
      deltaPosX = deltaPosX + deltaC * cos(alpha * PI / 180);
      deltaPosY = deltaPosY + deltaC * sin(alpha * PI / 180);
      alpha = (deltaAlpha + alpha);
#if defined(debugLocalizationOn)
      Serial.print("delta alpha:");
      Serial.print(deltaAlpha);
      Serial.print(" deltaC:");
      Serial.print(deltaC);
      Serial.print(" ");
      Serial.print(" alpha:");
      Serial.println(alpha);

      //     deltaPosX = deltaPosX + deltaC * (1 - (deltaAlpha * deltaAlpha) / 2);
      //     deltaPosY = deltaPosY + deltaC * deltaAlpha;
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
    bitWrite(currentMove, toDoRotation, false) ;
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
    /*
    if (bitRead(toDo, toDoBackward) == true )
    {
      deltaPosX = -deltaPosX;
      deltaPosY = -deltaPosY;

    }
    */
    if (bitRead(currentMove, toDoStraight) == true)
    {
      posX = posX + deltaPosX;
      posY = posY + deltaPosY;
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
void stopWheelControl()       // stop monitoring wheels rotation
{
#if defined(debugWheelControlOn)
  Serial.println("stopwheelcontrol");
  PrintSpeed();
#endif
  ComputeNewLocalization(0xff);
  currentMove = currentMove & 0xf9;
  StopRightWheelSpeedControl();
  StopLeftWheelSpeedControl();

}

void PrintSpeed()
{
  Serial.print("leftAvgSpeed:");
  Serial.print(avgLeftWheelSpeed);
  Serial.print(" rightAvgSpeed:");
  Serial.print(avgRightWheelSpeed);
  delayPrintSpeed = millis();
  Serial.print(" leftCentiDone:");
  Serial.print(((leftWheelInterrupt * 100) / leftWheelEncoderHoles) * iLeftMotorDemultiplierPercent / 100);
  Serial.print(" rightCentiDone:");
  Serial.println(((rightWheelInterrupt * 100) / rightWheelEncoderHoles) * iRightMotorDemultiplierPercent / 100);
}


void EchoServoAlign()    // to align echo servo motor with robot
{
  myservo.attach(servoPin);
  myservo.write(pulseValue[(nbPulse + 1) / 2]);  // select the middle of the pulse range
  delay(1000);
  // myservo.detach();
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
  }

}

void SendStatus()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x65; //
  PendingDataReqSerial[1] = AppStat; //
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
  PendingDataLenSerial = 0x11; // 6 longueur mini pour la gateway
}
void SendPowerValue()
{
  //  Serial.print("power1:");
  power1Mesurt = 3 * map(analogRead(power1Value), 0, 1023, 0, 467);
  //  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 12v
  // Serial.println("cVolt");
  //Serial.print(" power2:");
  if (power1Mesurt < power1LowLimit)
  {
    bitWrite(diagPower, 0, 1);
  }
  else
  {
    bitWrite(diagPower, 0, 0);
  }
  power2Mesurt = 2 * map(analogRead(power2Value), 0, 1023, 0, 456);
  //  Serial.print(power2Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  //  Serial.print("cV ");
  if (power2Mesurt < power2LowLimit)
  {
    bitWrite(diagPower, 1, 1);
  }
  else
  {
    bitWrite(diagPower, 1, 0);
  }
  power3Mesurt = 2 * map(analogRead(power3Value), 0, 1023, 0, 540);
  if (power3Mesurt < power3LowLimit)
  {
    bitWrite(diagPower, 2, 1);
  }
  else
  {
    bitWrite(diagPower, 2, 0);
  }
  //  Serial.print( "power3:");
  //  Serial.print(power3Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  //  Serial.println("cV");
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  PendingDataReqSerial[3] = uint8_t(power3Mesurt / 10);
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = 0x00;
  PendingDataLenSerial = 0x06; // 6 longueur mini pour la gateway

}
void StopAll()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  toDo = 0x00;
  myservo.detach();
  SendStatus();
}
void StartEchoInterrupt()
{
#if defined(debugObstacleOn)
  Serial.println("start echo interrupt");
#endif
  prevMicros = 0;
  lastMicro = 0;
  trigOn = false;
  scanNumber = 0;
  attachInterrupt(digitalPinToInterrupt(echoCurrent), EchoInterrupt, CHANGE);
}
void StopEchoInterrupt()
{
#if defined(debugObstacleOn)
  Serial.println("stop echo interrupt");
#endif
  detachInterrupt(digitalPinToInterrupt(echoCurrent));
  echoCurrent = 0;
  trigCurrent = 0;
  prevMicros = 0;
  lastMicro = 0;
  trigOn = false;
}
void EchoInterrupt()
{
  // Serial.println(micros());
  if (digitalRead(echoCurrent) == 1)
  {
    prevMicros = micros();
  }
  else
  {
    //      Serial.print("change:");
    lastMicro = micros() - prevMicros;
  }
  //   detachInterrupt(echoFront);
  //  attachInterrupt(echoFront,echoOut,FALLING);
}


void CheckObstacle()
{
  if (trigOn == true )
  {
    boolean switchPause = bitRead(waitFlag, toDoPause);
    if (scanNumber > 1)   // first echo non significative
    {
      unsigned int dist = lastMicro / 58;
      //      Serial.println(trigCurrent);
      //      Serial.println(toDo, HEX);
      int deltaSecurity = 0;
      if (trigCurrent == trigFront)

      {
        deltaSecurity = frontLenght + securityLenght;
      }
      else
      {
        deltaSecurity = backLenght + securityLenght;
      }

      if (dist < deltaSecurity)
      {
        bitWrite(waitFlag, toDoPause, 1);       // position bit pause on
      }
      else
      {
        bitWrite(waitFlag, toDoEndPause, 1);     // position bit pause to end
      }
      //     Serial.println(toDo, HEX);


#if defined(debugObstacleOn)
      Serial.print("dist obstacle:");
      Serial.print(scanNumber);
      Serial.print(" ");
      Serial.println(dist);
      if (bitRead(waitFlag, toDoPause) == 1 && switchPause == false)
      {
        Serial.println("pause due to obstacle");
      }
#endif
      if (bitRead(waitFlag, toDoPause) == 1 && switchPause == false)
      {
        PauseMove();
      }
    }
  }

  else {
    scanNumber++;
    digitalWrite(trigCurrent, LOW);  //
    delayMicroseconds(2);      //
    digitalWrite(trigCurrent, HIGH);
    delayMicroseconds(15); // 10 micro sec mini
  }
  trigOn = !trigOn;
  checkObstacleTimer = millis();
}
void PauseMove()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  bitWrite(diagRobot, 0, 1);       // position bit diagRobot
  Serial.println("stop obstacle");
  delayToStopWheel = millis();
  bitWrite(pendingAction, pendingLeftMotor, false);
  bitWrite(pendingAction, pendingRightMotor, false);
  ComputeNewLocalization(0xff);
  deltaPosX = 0;
  deltaPosY = 0;
}

void ResumeMove()  // no more obstacle resume moving
{

  Serial.println("resume move");
  long deltaX = targetX - posX;   // compute move to do to reach target
  //  Serial.println(deltaX);
  long deltaY = targetY - posY; // compute move to do to reach target
  //  Serial.println(deltaY);
  float deltaAlpha = atan(deltaY / deltaX); // compute move to do to reach target
  int rotation = alpha - deltaAlpha;
  int lenToDo = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  if (targetX < posX)
  {
    lenToDo = -lenToDo;
  }
#if defined(debugLocalizationOn)
  Serial.print("rotation:");
  Serial.print(rotation);
  Serial.print(" dist:");
  Serial.println(lenToDo);
#endif
  if (rotation != 0)
  {
    bitWrite(toDo, toDoRotation, 1);
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
  Move(rotation, lenToDo);
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
      bitWrite(waitFlag, toDoWait, 1);       // position bit pause on
      //     PauseMove();
    }
  }
  else
  {
    newDist = PingFront();
    if (abs(newDist) < frontLenght + securityLenght)
    {
      bitWrite(waitFlag, toDoWait, 1);       // position bit pause on
      //     PauseMove();
    }
  }
}


void CheckEndOfReboot()
{
  if (reboot == true && millis() > rebootDuration)  // end of arduino reboot
  {
    diagRobot = 0x00;
    reboot = false;
  }
}

void ComputeAverageWheelsSpeed()
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

void DetectEndOfMove()
{
  if (bitRead(saveCurrentMove, toDoRotation) == true && bitRead(currentMove, toDoRotation) == false && bitRead(currentMove, toDoStraight) == false)
  {
#if defined(debugMoveOn)
    Serial.println("end of rotation only");
#endif
    ComputeNewLocalization(0x01);  // compute after end of rotation
    //    stopWheelControl();
  }
  if (bitRead(saveCurrentMove, toDoStraight) == true && bitRead(currentMove, toDoStraight) == false )
  {
#if defined(debugMoveOn)
    Serial.println("end of straight");
#endif
    ComputeNewLocalization(0xff); // compute after end of straight move
    StopEchoInterrupt();          // stop obstacle detection
    // stopWheelControl();
  }
  if (currentMove == 0x00)        // no more rotation or move to do
  {
    //  ComputeNewLocalization(0xff);
#if defined(debugMoveOn)
    Serial.println("eofmove");
#endif
    StopEchoInterrupt();        // stop obstacle detection
    actStat = 0x69;            // status move completed
  }
  saveCurrentMove = currentMove;
}

























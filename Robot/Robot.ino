String Version = "RobotV1";
// uncomment #define debug to get log on serial link
//#define debugScanOn true
#define debugMoveOn true
//#define debugObstacleOn true
#define debugLocalizationOn true
#define debugMotorsOn true
#define debugWheelControlOn true
//#define debugConnection true
#define debugPowerOn true
#define wheelEncoderDebugOn true
#include <Servo.h>  // the servo library use timer 5 with atmega
#include <math.h>
#include <EEPROM.h>  // 
#include <Stepper.h>
#include <SoftwareSerial.h>
#include <EchoObstacleDetection.h>
#include <WheelControl.h>
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
int iLeftWheelDiameter = 64; //(in mm - used to measure robot moves)  65mn a vid
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
unsigned int iLeftTractionDistPerRev =  (PI * iLeftWheelDiameter) ;
unsigned int iRightTractionDistPerRev = (PI * iRightWheelDiameter);
int iRobotWidth = 455; // distance beetwen the 2 wheels mm
float coeffGlissementRotation = 1.;
#define frontLenght  35 // from echo system
#define backLenght  12 // from echo system
#define securityLenght 50 // minimal obstacle distance 
#define rebootDuration 10000 // delay to completly start arduino

//-- power control --
int uRefMotorVoltage = 1200; // mVolt for maxRPM
//#define power1Pin 53  // power 1 servomoteur
#define power1Value A15  // 12v power for motors
#define power2Value A14  // 5v power for arduino mega
//#define power3Value A13  // 9V power for esp8266 and components
#define power1LowLimit 720   // minimum centi volt before warning
#define power2LowLimit 420   // minimum centi volt before warning
//#define power3LowLimit 750  // minimum centi volt before warning
int power1Mesurt = 0;   // current power1 value
int power2Mesurt = 0;   // current power2 value
//int power3Mesurt = 0;   // current power3 value

//-- left Motor connection --
#define leftMotorENA 12 // Arduino pin must be PWM  use timer 3
#define leftMotorIN1 26 // arduino pin for rotation control
#define leftMotorIN2 27  // arduino pin for rotation control
#define leftMotorId 0
//-- right Motor connection --
#define rightMotorENB 11 // Arduino pin must be PWM use timer 3
#define rightMotorIN3 25 // arduino pin for rotation control
#define rightMotorIN4 24  // arduino pin for rotation control
#define rightMotorId 1
//-- motors control --
#define iLeftSlowPMW 125        // PMW value to slowdown motor at the end of the run
#define pendingLeftMotor 0      // define pendingAction bit used for left motor
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPMW); // define left Motor
#define iRightSlowPMW iLeftSlowPMW   // PMW value to slowdown motor at the end of the run
#define pendingRightMotor 1    // define pendingAction bit used for right motor
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPMW); // define right Motor

//-- wheel control --
#define wheelPinInterrupt 2    // used by sotfware interrupt when will rotation reachs threshold
#define leftWheelEncoderHoles 8  // number of holes of the encoder wheel
#define rightWheelEncoderHoles leftWheelEncoderHoles // number of holes of the encoder wheel
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A9  // analog input right encoder
#define leftWheelId 0           // to identify left wheel Id 
#define rightWheelId 1         // to identify right wheel Id 
int iLeftRevSpeed;              // instant left wheel speed
int iRightRevSpeed;             // instant right wheel speed
#define sizeOfLeftRev 8 // size of the array containing latest revolution wheel speed
#define sizeOfRightRev 8 // size of the array containing latest revolution wheel speed
unsigned long saveLeftWheelInterrupt = 0;             // copy of previous number of interrupts for the left encoder
unsigned long saveRightWheelInterrupt = 0;            // copy of previous number of interrupts for the right encoder
int iRightSpeed = 408;                                     // default expected robot speed mm/s (408 maxi)
int iLeftSpeed = (iRightSpeed * 100) / 100;                 // default expected robot speed mm/s (408 maxi) adjust left and right according to motors performance in order to go straight
// to adjust low and high value set leftWheelControlOn true, rotate left wheel manualy and read on serial the value with and wihtout hole
// must fit with the electonic characteristic
#define leftIncoderHighValue 350   // define value above that signal is high 
#define leftIncoderLowValue 150    // define value below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
#define rightIncoderHighValue 550  // define value above that signal is high
#define rightIncoderLowValue 350   // define value below that signal is low
#define delayBetweenEncoderAnalogRead  750 //  micro second between analog read of wheel encoder level
// create wheel control object
WheelControl Wheels(leftWheelEncoderHoles, leftIncoderHighValue, leftIncoderLowValue, leftAnalogEncoderInput,
                    rightWheelEncoderHoles, rightIncoderHighValue , rightIncoderLowValue, rightAnalogEncoderInput,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    wheelPinInterrupt);
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
#define toWait 0        // wait before moving
#define toEndPause 1     // could restart
#define toPause 2     // move to be temporaly paused
#define minDistToBeDone 30  // mn
#define minRotToBeDone 5  // degree
//unsigned long iDistance = 0; // mm
int pendingStraight = 0;   // copy of straight distance to be done
int reqAng = 0;          // requested rotation
int reqMove = 0;         // requested move
uint8_t resumeCount = 0; // nb of echo check before resume move
int movePingInit;     // echo value mesured before moving

//-- scan control --
#define servoPin 28    //  stepper
#define toDoScan 0    // toDo bit for scan request
#define echoFront 20  // arduino pin for mesuring echo delay for front 
#define echFrontId 0 //
#define trigFront  22     // arduino pin for trigerring front echo
#define echoBack  21   // arduino pin for mesuring echo delay for back 
#define trigBack  23    // arduino pin for trigerring back echo
#define echBacktId 1 //
#define nbPulse 15    // nb of servo positions for a 360Â° scan
#define echo3 false  // does not exit
#define echo3Alert false  // does not exit
#define echo4 false  // does not exit
#define echo4Alert false  // does not exit
#define echoPinInterrupt 18 // pin 18 dedicated to software usage 
boolean toDoEchoFront = true;
boolean echoFrontAlertOn = true; // does not exit
boolean toDoEchoBack = true;
boolean echoBackAlertOn = true; // does not exit
float echoCycleDuration = 0.5;
EchoObstacleDetection echo(echoFront, trigFront, echoBack, trigBack, 0, 0, 0, 0, echoPinInterrupt);
int numStep = 0;     // current scan step number
int nbSteps = 0;     // number of steps to be done for a scan
boolean switchFB = 0;  // switch front back scan
//float coefAngRef;
#define nbPulse 15    // nb of servo positions for a 360Â° scan
int pulseValue[nbPulse] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165}; // corresponding to 360Â° split in 15 steps
float coefAngRef = PI / (pulseValue[14] - pulseValue[0]);  // angle value beetwen 2 pulses
uint8_t pulseNumber = 0;  // pointer to pulseValue array
int distFSav = 0;     // saved front echo distance in cm
int distBSav = 0;      // saved back echo distance in cm
int valAng;  // servo orientation
float AngleRadian;  // echo servo orientation in radian
float AngleDegre;    // echo servo orientation in degre
int scanOrientation = 0;
//volatile unsigned long prevFrontMicros = 0; // used during move to dynamicaly avoid obstacles
//volatile unsigned int lastFrontMicro = 0; // used during move to dynamicaly avoid obstacles
//volatile unsigned long prevBackMicros = 0; // used during move to dynamicaly avoid obstacles
//volatile unsigned int lastBackMicro = 0; // used during move to dynamicaly avoid obstacles
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
unsigned long timeBetweenOnOffObstacle;  // use to compute delay between obstacle detection sitch on off
#define delayBetween2Scan 1500  // delay between to scan steps - must take into account the transmission duration
unsigned long leftReadAnalogTimer;
unsigned long leftTimerInt;
unsigned long timerLeftWheelPrint;
unsigned long rightReadAnalogTimer;
unsigned long rightTimerInt;
unsigned long timerRightWheelPrint;
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
uint8_t sendInfoSwitch = 0x00;
uint8_t saveCurrentMove = 0x00;   // flag last kind of current move
boolean moveForward;           // flag to indicate if requested move is forward or backward
uint8_t appStat = 0xff; // statut code application 00 active ff stopped 1er demi octet mouvement 2eme demi scan
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
  //  pinMode(power1Pin, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(wheelPinInterrupt, OUTPUT);
  digitalWrite(blueLed, HIGH);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, HIGH);
  //  myservo.attach(servoPin);                       // attach the echo servo
  EchoServoAlign();                               // adjust servo motor to center position
}

void loop() {
  CheckEndOfReboot();
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
  // *** end loop keep in touch with the server

  // *** power check
  if ((millis() - timePower1Check) >= delayPowerCheck && toDo == 0x00 ) // regularly check power values
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
      Move(reqAng, reqMove);                    // move according to the request first rotation and the straight move
      appStat = appStat | 0xf0;
    }
    // *** end of checking resqueted actions

    // *** wheels speed control and localization

    if (millis() - delayCheckPosition > 200  && bitRead(currentMove, toDoStraight) == true && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    { // Compute new robot position
      delayCheckPosition = millis();  // reset timer
      //  CheckMoveSynchronisation();
      ComputeNewLocalization(0x00);   // dynamic localization
    }
    //#if defined(debugWheelControlOn)
    if (millis() - delayPrintSpeed > 1000 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
    {
      PrintSpeed();
    }
    //#endif
    // *** end of loop wheels control speed and localization

    // *** checking for obstacles
    if (bitRead(waitFlag, toPause) == 1 && (millis() - checkObstacleTimer) > 750 ) // robot in pause status
    {
      CheckNoMoreObstacle();
      checkObstacleTimer = millis();
      if ( bitRead(waitFlag, toEndPause) == 1 ) // no more obstacle
      {
        resumeCount++;
        if (resumeCount > 3)
        {
          ResumeMove();
          resumeCount = 0;
          bitWrite(waitFlag, toPause, 0);
          bitWrite(waitFlag, toEndPause, 0);
          bitWrite(waitFlag, toWait, 0);       //
        }
      }

    }

    // *** end of loop checking obstacles

    // *** checking move completion

    if (bitRead(toDo, toDoStraight) == true && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false)
    {
      if ( bitRead(currentMove, toDoRotation) == true )  // end of first move step (rotation) move forward to be done
      {
#if defined(debugMoveOn)
        Serial.println("end rotation still straight move todo");
#endif
        //      delayToStopWheel = 0; // to avoid pending wheel stopping
        //      ComputeNewLocalization(0x01);  // compute new localization after end of rotation

      }
      MoveForward(pendingStraight) ;
    }
    //  if (delayToStopWheel != 0 && (millis() - delayToStopWheel) > 80 && toDo == 0x00 ) // delay to completely stop robot
    if (delayToStopWheel != 0 && (millis() - delayToStopWheel) > 80  ) // delay to completely stop robot
    {
      //   stopWheelControl();
      delayToStopWheel = 0;
      SendStatus();
    }
    // *** end of checking move completion

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
    case 0x6d: // commande m means move
      if (appStat != 0xff)
      {
        appStat = appStat & 0x1f;
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
        }
        if (reqMove < 0)
        {
          bitWrite(toDo, toDoStraight, true);
          bitWrite(toDo, toDoBackward, true);  // to go backward
        }
#if defined(debugMoveOn)
        Serial.print(" ");
        Serial.println(reqMove);
        Serial.print("todo 0x");
        Serial.print(toDo, HEX);
        Serial.print(" waitflag 0x");
        Serial.println(waitFlag, HEX);
#endif
      }
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
      }
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
void InitScan(int nbS, int startingOrientation) {
  numStep = 0;
  nbSteps = nbS;
  scanOrientation = startingOrientation;
  //  digitalWrite(power1Pin, HIGH);
  EchoServoAlign();
  myservo.attach(servoPin);
}

void ScanPosition() {

  // if ((millis() - timeScanFront) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 )
  if ((millis() - timeScanBack) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 && pendingAckSerial == 0x00)
  {

    timeScanFront = millis();
    switchFB = 1;
    distFSav = ScanOnceFront(numStep);
    //      vw_rx_start();        // dÃƒÆ’Ã‚Â©marrer la rÃƒÆ’Ã‚Â©ception
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


  actStat = 0x67; //" scan completed
  PendingDataReqSerial[0] = actStat;
  PendingDataReqSerial[1] = appStat;
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
  // digitalWrite(power1Pin, LOW);
  numStep = 0;

}
void Move(int orientation, int lenghtToDo)
{

  ComputeTargetLocalization(orientation, lenghtToDo);
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
  trigCurrent = trigFront;
  trigBoth = true;         // need to scan front and back during rotation for obstacle detection
  echoCurrent = echoFront; // start echo front
  //  StartEchoInterrupt(1, 1);
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
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, iLeftSpeed, lentghRightToDo, iRightSpeed);
    //    bitWrite(currentMove, toDoRotation, true);
    //    bitWrite(saveCurrentMove, toDoRotation, true);
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
    ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, iLeftSpeed, lentghRightToDo, iRightSpeed);
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
      //   StartEchoInterrupt();
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
      //    StartEchoInterrupt();
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
  power1Mesurt = 2 * map(analogRead(power1Value), 0, 1023, 0, 1080);

  //  Serial.print("power2:");
  if (power1Mesurt < power1LowLimit)
  {
    bitWrite(diagPower, 0, 1);
  }
  else
  {
    bitWrite(diagPower, 0, 0);
  }
  power2Mesurt = 2 * map(analogRead(power2Value), 0, 1023, 0, 490);
#if defined(debugPowerOn)
  Serial.print(analogRead(power1Value));
  Serial.print(" ");
  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 9v
  Serial.println("cV 1");
  Serial.print(analogRead(power2Value)); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print(" ");
  Serial.print(power2Mesurt);
  Serial.println("cV 2");
#endif
  if (power2Mesurt < power2LowLimit)
  {
    bitWrite(diagPower, 1, 1);
    myservo.detach();
  }
  else
  {
    bitWrite(diagPower, 1, 0);
  }

  if (diagPower >= 0x03 && appStat != 0xff)
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
    //  bitWrite(diagMotor, 0, 0);       // position bit diagMotor left motor
    //  bitWrite(diagMotor, 1, 0);       // position bit diagMotor right motor
    diagMotor = 0x00;
    saveLeftWheelInterrupt = 0;
    saveRightWheelInterrupt = 0;
    digitalWrite(wheelPinInterrupt, LOW);
    attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);


    Wheels.StartWheelControl(true, true, iLeftCentiRevolutions , true, true, iRightCentiRevolutions , false, false, 0, false, false, 0);
    //StartLeftWheelSpeedControl();   //
    // StartRightWheelSpeedControl();   //
    rightMotor.RunMotor(bRightClockwise,  iRightRevSpeed);
    leftMotor.RunMotor(bLeftClockwise,  iLeftRevSpeed);
  }
}

void ComputerMotorsRevolutionsAndrpm(unsigned long iLeftDistance, int iLeftSpeed, unsigned long iRightDistance, int iRightSpeed)
{

  float powerAdustment = float (uRefMotorVoltage) / (3 * map(analogRead(power1Value), 0, 1023, 0, 467)); // speed adjustement to power supply
  iLeftCentiRevolutions = (iLeftDistance * leftWheelEncoderHoles / iLeftTractionDistPerRev) ; // Centi-revolutions
  iRightCentiRevolutions = (iRightDistance * rightWheelEncoderHoles / iRightTractionDistPerRev) ; // ms

  iLeftRevSpeed = (iLeftSpeed * 60 ) / iLeftTractionDistPerRev; // revolutions per minute
  iRightRevSpeed = (iRightSpeed * 60) / iRightTractionDistPerRev; // revolutions per minute
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
  if ( deltaMove > 2 && deltaMove / Wheels.GetCurrentHolesCount(0) > 0.05)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    bitWrite(diagMotor, 3, 1);       // position bit diagMotor
    Serial.print("move synchro pb:");
    Serial.print(deltaMove);
    Serial.print(" ");
    Serial.println(abs(deltaMove / Wheels.GetCurrentHolesCount(0)));
    if (bitRead(currentMove, toDoRotation) == true)
    {
      ComputeNewLocalization(0x01);
    }
    else {
      ComputeNewLocalization(0xff);
    }

    // toDo = 0x00;
  }
}
void ComputeNewLocalization(uint8_t param)  // compute localization according to the wheels rotation
{
  unsigned int currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  float deltaAlpha = 0; // modification of robot angles (in degres) since last call of the function
  float deltaLeft = ((float (currentLeftHoles) - saveLeftWheelInterrupt) * PI * iLeftWheelDiameter / leftWheelEncoderHoles); // number of centimeters done by left wheel since last call of the function
  float deltaRight = ((float (currentRightHoles) - saveRightWheelInterrupt) * PI * iRightWheelDiameter / rightWheelEncoderHoles); // number of centimeters done by right wheel since last call of the function

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
    //   bitWrite(currentMove, toDoRotation, false) ;
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
/*
void stopWheelControl()       // stop monitoring wheels rotation
{
#if defined(debugWheelControlOn)
  Serial.println("stopwheelcontrol");
  PrintSpeed();
#endif
  if (bitRead(currentMove, toDoRotation) == true)
  {
    ComputeNewLocalization(0x01);
  }
  else {
    ComputeNewLocalization(0xff);
  }
  currentMove = currentMove & 0xf9;
  StopRightWheelSpeedControl();
  StopLeftWheelSpeedControl();

}
*/
void PrintSpeed()
{
  delayPrintSpeed = millis();
  Serial.print("leftRPM:");
  Serial.print(Wheels.GetLastTurnSpeed(0) * 60);
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(0) * 60);
  Serial.print(" rightRPM:");
  Serial.print(Wheels.GetLastTurnSpeed(0) * 60);
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(1) * 60);
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
  PendingDataLenSerial = 0x11; // 6 longueur mini pour la gateway
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
  PendingDataLenSerial = 0x06; // 6 longueur mini pour la gateway

}
void StopAll()
{
  leftMotor.StopMotor();
  rightMotor.StopMotor();
  myservo.detach();
  StopEchoInterrupt(true, true);
  Wheels.StopWheelControl(true, true, 0, 0);
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
  //    digitalWrite(echoPinInterrupt, LOW);
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
      }
    }
  }


  //     Serial.println(toDo, HEX);
  checkObstacleTimer = millis();  // reset timer

#if defined(debugObstacleOn)
  // Serial.print(" obstacle:");
  // Serial.print(scanNumber);
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
  bitWrite(diagRobot, 0, 1);       // position bit diagRobot
  Serial.println("stop obstacle");
  //  delayToStopWheel = millis();
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
  Serial.println("resume move");
  long deltaX = targetX - posX;     // compute move to do to reach target
  //  Serial.println(deltaX);
  long deltaY = targetY - posY;     // compute move to do to reach target
  //  Serial.println(deltaY);
  float deltaAlpha = atan(deltaY / deltaX) * 180 / PI; // compute move to do to reach target
  int rotation = alpha - deltaAlpha;
  int lenToDo = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  if (targetX < posX)
  {
    lenToDo = -lenToDo;
  }
#if defined(debugLocalizationOn)
  Serial.print("deltaxy:");
  Serial.print(deltaX);
  Serial.print(" ");
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
  Wheels.ClearThershold(wheelId);                      // clear the threshold flag to avoid more interruption
  WheelThresholdReached(wheelId);                      // call the threshold analyse

#if defined(debugMoveOn)
  Serial.print("wheel int:");
  Serial.println(Wheels.GetLastWheelInterruptId());
#endif

}

void WheelThresholdReached( uint8_t wheelId)
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
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
#if defined(debugMoveOn)
  Serial.print("RPM:");
  Serial.print(wheelId);
  Serial.print(" ");
  Serial.print(Wheels.GetLastTurnSpeed(wheelId) * 60);
  Serial.print(" ");
  Serial.println(Wheels.Get2LastTurnSpeed(wheelId) * 60);
  Serial.print("Holesleft:");
  Serial.print(Wheels.GetCurrentHolesCount(leftWheelId));
  Serial.print(" right:");
  Serial.println(Wheels.GetCurrentHolesCount(rightWheelId));
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
  if (bitRead(toDo, toDoStraight) == false)             // no more move to do
  {
    actStat = 0x69;                                      // status move completed
    toDo = 0x00;                                         // clear flag todo
  }
}





















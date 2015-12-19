String Version = "RobotV0-4";
// passage sur GitHub
//#include <ServoTimer2.h>
#include <Servo.h>  // the servo library
//ServoTimer2 myservo;  // create servo object to control a servo
Servo myservo;  // create servo object to control a servo
//#include <VirtualWire.h>
//#include <NetworkVariable.h>   // needed for RF434 network
//#include <NetworkVoid.h>  // needed for RF434 network
#include <SerialNetworkVariable.h>   // needed for RF434 network
#include <SerialNetworkVoid.h>  // needed for RF434 network
#include <motorControl2342L.h>
uint8_t PendingReqRef = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRef = 0xbf; // pending request (if 0x00 none)- copy fo retry
uint8_t PendingReqRefSerial = 0xbf; // pending request (if 0x00 none)
uint8_t PendingSecReqRefSerial = 0xbf; // pending request (if 0x00 none)- copy fo retry
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
#include <EEPROM.h>
#include <Stepper.h>
//#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
//-- General Robot parameters --
int iLeftMotorMaxrpm = 120; // (Value unknown so far - Maximum revolutions per minute for left motor)
int iRightMotorMaxrpm = iLeftMotorMaxrpm ; // (Value unknown so far - Maximum revolutions per minute for left motor)
float fMaxrpmAdjustment;  // will be used to compensate speed difference betweeen motors
int iLeftWheelDiameter = 65; //(in mm - used to measure robot moves)
int iRightWheelDiameter = iLeftWheelDiameter; //(in mm - used to measure robot moves)
float iLeftMotorDemultiplierPercent = 100; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel) used to fit speed encoder rotation to wheel rotation
float iRightMotorDemultiplierPercent = iLeftMotorDemultiplierPercent; // (1 revolution of motor correspons to ileftMotorDemultiplierPercent/100 revolutions of wheel)
unsigned int iLeftTractionDistPerRev =  (PI * iLeftWheelDiameter) * 100 / (iLeftMotorDemultiplierPercent);
unsigned int iRightTractionDistPerRev = (PI * iRightWheelDiameter) * 100 / (iRightMotorDemultiplierPercent);
int iRobotWidth = 450; // distance beetwen the 2 wheels mm
int uRefMotorVoltage = 1200; // mVolt for maxRPM
//int rayonRoue = 37; //en mn
//-- left Motor connection --
int leftMotorENA = 6; //ConnectÃ© Ã  Arduino pin (sortie pwm)
int leftMotorIN1 = 5; //ConnectÃ© Ã  Arduino pin
int leftMotorIN2 = 7; //ConnectÃ© Ã  Arduino pin
#define wheelSpeedLeftPin 19   // pin
#define leftWheelEncoderHoles 8
//-- right Motor connection --
int rightMotorENB = 3; //ConnectÃ© Ã  Arduino pin (Sortie pwm)
int rightMotorIN3 = 2; //ConnectÃ© Ã  Arduino pin
int rightMotorIN4 = 4; //ConnectÃ© Ã  Arduino pin
#define wheelSpeedRightPin 18  // pin 
#define rightWheelEncoderHoles leftWheelEncoderHoles
float avgRightWheelSpeed = 0;
float avgLeftWheelSpeed = 0;
//boolean bClockwise = true; //Used to turn the motor clockwise or counterclockwise
# define bForward  true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
//
unsigned long iDistance = 0; // mm
int iSpeed = 408; // mm/s 408 maxi
unsigned long iLeftCentiRevolutions;
unsigned long iRightCentiRevolutions;
unsigned long restartedDelay;
int iLeftRevSpeed;
int iRightRevSpeed;
#define sizeOfLeftRev 8
#define sizeOfRightRev 8
unsigned int instantLeftWheelRevSpeed[sizeOfLeftRev];
unsigned int instantRightWheelRevSpeed[sizeOfRightRev];
uint8_t leftWheelSpeedCount = 0x00;
uint8_t rightWheelSpeedCount = 0x00;
#define iLeftSlowPMW 120 // PMW value to slowdown motor at the end of the run
#define iRightSlowPMW iLeftSlowPMW   // 
Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm, iLeftSlowPMW);
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm, iRightSlowPMW);
//
unsigned long prevLeftWheelInterrupt = 0;
volatile unsigned long leftWheelInterrupt = 0;
unsigned long prevRightWheelInterrupt = 0;
volatile unsigned long rightWheelInterrupt = 0;
unsigned long saveLeftWheelInterrupt = 0;
unsigned long saveRightWheelInterrupt = 0;
unsigned long timeAppli; // cycle applicatif
unsigned long timeStatut;  // cycle envoi du statut au master
unsigned long timeNetwStat; // cycle affichage des stat RF433 sur serial
unsigned long timeRTC;  // cycle demande heure
unsigned long timeScanFront;  // cycle demande heure
unsigned long timeScanBack;  // cycle demande heure
unsigned long timeSendSec;  //
unsigned long timeSendSecSerial;  //
unsigned long timeReceiveSerial;  //
unsigned long timePower1Check;  //
unsigned long delayToStopWheel = 0;
unsigned long prevLeftWheelIntTime;
unsigned long prevRightWheelIntTime;
unsigned long delayCheckSpeed;
unsigned long delayCheckPosition;
unsigned long delayPrintSpeed;
unsigned long timeAffLed;
unsigned long serialAliveTimer;  //  for serial link communication
unsigned long serialTimer;       //   for serial link communication
unsigned long checkObstacleTimer;     // to regurarly ckeck obstacle when moving
unsigned long durationMaxEcho = 25000; // en us
unsigned long timeSendInfo;
uint8_t diagMotor = 0x00;   // bit 0 pb left motor, 1 right moto, 2 synchro motor
uint8_t diagPower = 0x00;   // bit 0 power1, 1 power2, 2 power3 0 Ok 1 Ko
uint8_t diagConnection = 0x01;   // bit 0 pending serial link
uint8_t diagRobot = 0x00;
uint8_t toDo = 0x00; //
uint8_t pendingAction = 0x00; //
uint8_t sendInfoSwitch = 0x00;
boolean moveFront;
int movePingInit;
long deltaPosX = 0;
long deltaPosY = 0;
float alpha = 0;
long posX = 0;
long posY = 0;
long targetX = 0;
long targetY = 0;
//uint8_t checkPositionCount = 0;
#define pendingLeftMotor 0
#define pendingRightMotor 1
#define toDoScan 0
#define toDoMove 1
#define toDoRotation 2
#define toDoForward 3
#define delayBetween2Scan 1500 // pour laisser le temps de positionner le servo et remonter data
#define delayBetweenScanFB 700  // delai entre 2 pulses
#define delayBetweenInfo 5000
//uint8_t savDiagMotor = 0x00;
uint8_t nbRTC = 0;
int cpt = 0;
int numStep = 0;
int nbSteps = 0;
boolean switchFB = 0;
int bcl1 = 0; // definit le cycle d affichage des statistiques des stations actives
int Lbcl1 = 200; // limite bcl1
int bcl2 = 0; // definit le cycle d affichage des statistiques des stations actives
int Lbcl2 = 50; // limite bcl2
byte bcl6 = 0; // cycle retry send commande unitaire
byte bcl7 = 0; // cycle traitement consigne
int retryCount = 0;
String val;
String data = "";
char c = 0;
int distFSav = 0;
int distBSav = 0;
int pendingForward = 0;
/* Utilisation du capteur Ultrason HC-SR04 */
//
// definition des broches utilisees
#define echoFront 24
#define trigF  25
#define echoBack  22
#define trigB  23
#define delayPowerCheck 5000
int servoFeedbackPin = A0;
int servoFeedbackValue;
float AngleRadian;
float AngleDegre;
//uint8_t inc=50;  // increment de rotation
int inc = 50; // increment de rotation
unsigned long lecture_echo;
//uint16_t cm;
uint8_t trameNumber = 0;
uint8_t lastAckTrameNumber = 0;
uint8_t pendingAck = 0;
uint8_t pendingAckSerial = 0;
// a maximum of eight servo objects can be created
int pos;    // variable to store the servo position
//uint8_t Increment=1;
uint8_t AppStat = 0xff; // statut code application 00 active ff stopped 1er demi octet mouvement 2eme demi scan
uint8_t actStat = 0xff; // staut action en cours
int valAng;  // angle servo
int minAng = 950; // teste: min absolu 950 et maxi 2100 - pour un angle de 132Â°
int maxAng = 2100;
int reqAng = 0;
int reqMove = 0;
int addrEeprom = 0;
int orientation = 0; // reserved fo future usage
byte valueEeprom;
// moteur pas a pas
#define STEPS 64
#define servoPin 9
#define power1Pin 53  // power 1 servomoteur
#define power1Value A15  // 12v power
#define power2Value A14  // 9v power
#define power3Value A13  // 5V power
#define blueLed 50    // red LED
#define yellowLed 51    // red LED
#define greenLed 52  // green LED
#define redLed 53    // red LED
boolean blueLedOn =  true;
boolean yellowLedOn = true;
boolean greenLedOn =  true;
boolean redLedOn = true;
int power1Mesurt = 0;
int power2Mesurt = 0;
int power3Mesurt = 0;
int nbPas = 2048;
float coefAngRef;
int pulseValue[15] = {15, 26, 37, 47, 58, 69, 79, 90, 101, 112, 122, 133, 144, 154, 165};
//int pulseFeedback[15] = {294, 319, 344, 367, 389, 410, 435, 456, 482, 503, 525, 544, 565, 588, 611}; // valeurs relevees
int pulseFeedback[15] = {240, 258, 278, 298, 317, 336, 355, 373, 392, 409, 428, 445, 463, 481, 495}; // valeurs relevees avec alim a 580mv
int angleD = pulseFeedback[0];
#define nbPulse 15
uint8_t pulseNumber = 0;  // position de debut
float coeffGlissementRotation = 1.;
int i = 0;
// test steps 64 speed 110 stepper.step 2048 - 1 tour en 16,5s
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
//Stepper stepperD(STEPS, 6, 7); // moteur droit
//Stepper stepperG(STEPS, 8, 11); // moteur gauche 30 a cabler

//
void setup() {
  Serial.begin(38400);
  Serial2.begin(SpeedNetwSerial);
  Serial.println(Version);
  valueEeprom = EEPROM.read(addrEeprom);
  Serial.print("ID=");
  Serial.print(valueEeprom, DEC);
  Serial.println();
  //  if (valueEeprom != 0xff) {
  //    addrS = valueEeprom;
  //    addrSSerial = valueEeprom;
  //  }  // ecrase la valeur du programmne par celle lue en eeprom

  //  vw_set_tx_pin(3);   // choisir la broche 3 pour Ã©mettre
  //  vw_set_rx_pin(5);     // choisir la broche 5 pour recevoir
  //  vw_set_ptt_pin(4);     // choisir la broche 9 pour ptt - utilite a confirmer
  //  vw_setup(SpeedNetw);       // vitesse de reception  - tunned pour etre en synch avec le master ??
  //  vw_rx_start();        // dÃ©marrer la rÃ©ception
  // Serial.print("Speed=");
  // Serial.println(SpeedNetw);
  pinMode(trigF, OUTPUT);
  digitalWrite(trigB, LOW);
  pinMode(trigB, OUTPUT);
  digitalWrite(trigF, LOW);
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
  digitalWrite(power1Pin, LOW);
  Serial.print("nbpas: ");
  Serial.println(nbPas);
  coefAngRef = PI / (pulseValue[14] - pulseValue[0]);
  myservo.attach(servoPin);
  myservo.write(pulseValue[(nbPulse + 1) / 2]);
  delay(1000);
  myservo.detach();
  //  myservo.attach(servoPin);
  //  EchoServoAlign();
  //  myservo.detach();
  Serial.print("iLeftTractionDistPerRev:");
  Serial.println(iLeftTractionDistPerRev);
  PowerCheck();
}

void loop() {

  if (millis() - delayCheckSpeed > 250 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
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
  if (millis() - delayCheckPosition > 1000 && (prevLeftWheelInterrupt != leftWheelInterrupt || prevRightWheelInterrupt != rightWheelInterrupt) && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  {
    delayCheckPosition = millis();
    CheckMoveSynchronisation();
    boolean last = false;
    ComputeNewLocalization(last);
  }
  if (millis() - delayPrintSpeed > 1000 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  {
    PrintSpeed();

  }
  if (millis() - checkObstacleTimer > 1000 && (bitRead(pendingAction, pendingLeftMotor) == true || bitRead(pendingAction, pendingRightMotor) == true))
  {
    //    checkObstacle();
    checkObstacleTimer = millis();
  }
  if (bitRead(pendingAction, pendingLeftMotor) == true)
  {

    //   Serial.println(avgLeftWheelSpeed);
    int feedBackLeftMotor = leftMotor.CheckMotor( (avgLeftWheelSpeed * iLeftMotorDemultiplierPercent) / 100, ((leftWheelInterrupt * 100) / leftWheelEncoderHoles) * iLeftMotorDemultiplierPercent / 100 );
    switch (feedBackLeftMotor)
    {
      case -2:   // motor is stopped
        leftMotor.StopMotor();
        rightMotor.StopMotor();
        bitWrite(diagMotor, 0, 1);       // position bit diagMotor
        Serial.println("left motor pb");
        delayToStopWheel = millis();
        bitWrite(pendingAction, pendingLeftMotor, false);

        break;
      case -1:  // motor issue
        bitWrite(pendingAction, pendingLeftMotor, false);
        rightMotor.StopMotor();  // stop synchrone des 2 moteurs
        delayToStopWheel = millis();


        break;
      case 0:
        bitWrite(pendingAction, pendingLeftMotor, false);
        break;
      default:
        break;
    }

  }
  if (bitRead(pendingAction, pendingRightMotor) == true)
  {

    //    Serial.println(avgRightWheelSpeed );
    int feedBackRightMotor = rightMotor.CheckMotor( (avgRightWheelSpeed * iRightMotorDemultiplierPercent) / 100  , ((rightWheelInterrupt * 100) / rightWheelEncoderHoles) * iRightMotorDemultiplierPercent / 100 );
    switch (feedBackRightMotor)
    {
      case -2:
        rightMotor.StopMotor();
        leftMotor.StopMotor();
        bitWrite(diagMotor, 1, 1);       // position bit diagMotor
        Serial.println("right motor pb");
        delayToStopWheel = millis();
        bitWrite(pendingAction, pendingRightMotor, false);
        break;
      case -1:
        bitWrite(pendingAction, pendingRightMotor, false);
        leftMotor.StopMotor();  // stop synchrone des 2 moteurs
        delayToStopWheel = millis();

        break;
      case 0:
        bitWrite(pendingAction, pendingRightMotor, false);
        break;
      default:
        break;
    }


  }
  if (bitRead(toDo, toDoForward) == true && bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false)
  {
    MoveForward(pendingForward) ;
  }
  if (delayToStopWheel != 0 && (millis() - delayToStopWheel) > 1000) // delay to completely stop robot
  {
    Serial.println("stopwheelcontrol");
    stopWheelControl();
    delayToStopWheel = 0;
  }
  int getSerial = Serial_have_message();
  if (getSerial > 0)
  {

    TraitInput(DataInSerial[2]);
  }


  if (PendingReqSerial != 0x00 )
  {

    DataToSendSerial(); //
    timeSendSecSerial = millis();
  }

  if ((millis() - timeStatut) >= (295000 + random(0, 10000))) // toutes les 5mn
  {
    timeStatut = millis(); // au pas nÂ° 5000 tous les 1000
  }
  if ((millis() - timePower1Check) >= delayPowerCheck ) // toutes les 5mn
  {
    PowerCheck();

  }

  if (bitRead(toDo, toDoScan) == 1)
  {

    ScanPosition();

  }


  if (bitRead(toDo, toDoMove) == 1) {
    //  Serial.println("move");
    Move(reqAng, reqMove);
    AppStat = AppStat | 0xf0;
  }


  if (retryCount >= 3)
  {
    pendingAckSerial = 0x00;
    retryCount = 0;
  }
  if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 3) {
    ReSendSecuSerial();
    Serial.println("retry");
    timeSendSecSerial = millis();
    retryCount = retryCount + 1;
  }
  if (millis() - timeAffLed > 1000)
  {
    AffLed();
    timeAffLed = millis();
  }
  if (millis() - timeReceiveSerial >= 30000)
  {
    //   Serial.println(timeReceiveSerial);
    bitWrite(diagConnection, 1, 0);       // established connection
  }
  if (millis() - timeSendInfo >= delayBetweenInfo && toDo == 0x00)
  {
    if (sendInfoSwitch % 2 == 0)
    {
      SendStatus();
    }
    if (sendInfoSwitch % 2 == 1)
    {
      SendPowerValue();
    }
    sendInfoSwitch = sendInfoSwitch + 1;
    timeSendInfo = millis();
  }
}

void TraitInput(uint8_t cmdInput) {
  Serial.println("serialInput");
  bitWrite(diagConnection, 0, 0);       // established connection
  timeReceiveSerial = millis();

  switch (cmdInput) {

    case 0x73: // commande s
      Serial.println("cmd stop");
      AppStat = 0xff;
      leftMotor.StopMotor();
      rightMotor.StopMotor();
      toDo = 0x00;
      break;
    case 0x78: // commande x
      Serial.println("cmd start");
      AppStat = 0x00;
      break;
    case 0x65: // commande e
      Serial.println(" echo");
      SendStatus();

      break;
    case 0x41: // commande A

      break;
    case 0x42: // commande B

      break;
    case 0x49: // commande I
      Serial.println("init X Y");
      posX = 0;
      posY = 0;
      deltaPosX = 0;
      deltaPosY = 0;
      break;
    case 0x2b: // commande +
      AppStat = AppStat & 0xf1;
      Serial.println("Scan");
      pulseNumber = 0;
      InitScan(nbPulse, 0);
      actStat = 0x66;
      bitWrite(toDo, toDoScan, 1);       // position bit toDo scan
      //      SendRFNoSecured();
      break;
    case 0x6d: // commande m
      AppStat = AppStat & 0x1f;
      actStat = 0x68; // moving
      Serial.print("Move: ");
      bitWrite(toDo, toDoMove, 1);       // position bit toDo move
      reqAng = DataInSerial[4] * 256 + DataInSerial[5];

      if (DataInSerial[3] == 0x2d) {
        reqAng = -reqAng;
      }
      Serial.print(reqAng);
      reqMove = DataInSerial[7] * 256 + DataInSerial[8];
      if (DataInSerial[6] == 0x2d) {
        reqMove = -reqMove;
      }
      Serial.print(" ");
      Serial.println(reqMove);
      //   Serial.println(inc);
      break;
    case 0x53: // commande S
      Serial.print("Status: ");
      //Serial.println(inc);

      break;
    case 0x54: // commande T
      Serial.println("Demande Stat: ");
      //Serial.println(inc);

      break;
    case 0x61: // ack de trame du server
      Serial.print("acq: ");
      for (int i = 0; i < 5; i++)
      {
        Serial.print(DataInSerial[i], HEX);
        Serial.print(":");
      }
      Serial.println();
      lastAckTrameNumber = DataInSerial[3];
      //      Serial.print(":");
      Serial.print(lastAckTrameNumber);
      Serial.print(":");
      Serial.println(trameNumber);
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
  unsigned long time1;
  unsigned long time2;
  unsigned long deltaT;
  digitalWrite(trigF, HIGH);
  delayMicroseconds(15); // 10 micro sec mini
  time1 = micros();
  digitalWrite(trigF, LOW);
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
  unsigned long time1;
  unsigned long time2;
  unsigned long deltaT;
  /* for (int i=0;i>=sizeof(pulseValue);i++)
   {
   */
  //    PosServo();
  digitalWrite(trigB, HIGH);

  delayMicroseconds(15); // 10 micro sec mini
  time1 = micros();
  digitalWrite(trigB, LOW);
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
  orientation = startingOrientation;
  digitalWrite(power1Pin, HIGH);
  delay(500);
  myservo.attach(servoPin);  // attaches the servo on pin 4 to the servo object
  myservo.write(pulseValue[orientation]);
  delay(1000);

}

void ScanPosition() {

  // if ((millis() - timeScanFront) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 )
  if ((millis() - timeScanBack) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 && pendingAckSerial == 0x00)
  {

    timeScanFront = millis();
    switchFB = 1;
    distFSav = ScanOnceFront(numStep);
    //      vw_rx_start();        // dÃ©marrer la rÃ©ception
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
  // SendRFNoSecured();
}
void Move(int orientation, int lenghtToDo)
{
  bitWrite(toDo, toDoMove, false);       // position bit toDo move
  if (orientation != 0)
  {
    bitWrite(toDo, toDoRotation, true);       // position bit toDo move
    Rotate(orientation);
  }
  if (lenghtToDo != 0)
  {
    bitWrite(toDo, toDoForward, true);       // position bit toDo move
    pendingForward = lenghtToDo;
  }

}
void Rotate( int orientation) {
  bitWrite(toDo, toDoRotation, false);       // position bit toDo move
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
    if (orientation > 0)
    {
      bLeftClockwise = true;

    }
    else {
      bLeftClockwise = false;
    }
    bRightClockwise = bLeftClockwise;

    startMotors();
  }
  //
  Serial.println("rotate ");
  actStat = 0x69; //" move completed
}



// SendRFNoSecured();

void MoveForward( int lengthToDo) {
  EchoServoAlign();
  bitWrite(toDo, toDoForward, false);       // position bit toDo move
  unsigned int lentghLeftToDo = 0;
  unsigned int lentghRightToDo = 0;
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
      moveFront = false;
      movePingInit = PingBack();
    }
    else
    {
      bLeftClockwise = !bForward;
      bRightClockwise = bForward;
      moveFront = true;
      movePingInit = PingFront();
    }
    Serial.println(iLeftCentiRevolutions );
    startMotors();
  }

  Serial.println("move ok");

  actStat = 0x69; //" move completed
}




unsigned int  RotationCalculation(int orientation) {
  unsigned int distToDo = ((iRobotWidth * PI / 360) * orientation);
  Serial.print("distRot:");
  Serial.println(distToDo);
  return (distToDo);
}
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
    StopAll();
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
  StartLeftWheelSpeedControl();   //
  StartRightWheelSpeedControl();   //
  bitWrite(pendingAction, pendingLeftMotor, true);
  bitWrite(pendingAction, pendingRightMotor, true);
  Serial.print("reqSpeed:");
  Serial.print(iLeftRevSpeed);
  Serial.print(" ");
  Serial.println(iRightRevSpeed);
  //  bitWrite(diagMotor, 0, 0);       // position bit diagMotor left motor
  //  bitWrite(diagMotor, 1, 0);       // position bit diagMotor right motor
  diagMotor = 0x00;
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

  for (int i = 0; i < sizeOfLeftRev; i++)
  {
    instantLeftWheelRevSpeed[i] = 0 ; // init avec expected speed iLeftRevSpeed
  }
  avgLeftWheelSpeed;
  attachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin), LeftWheelCount, FALLING);
  leftWheelInterrupt = 0;
}
void StopLeftWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedLeftPin));
}
void LeftWheelCount()
{
  leftWheelInterrupt++;
}
void StartRightWheelSpeedControl()
{

  for (int i = 0; i < sizeOfRightRev; i++)
  {
    instantRightWheelRevSpeed[i] = 0; // init avec expected speediRightRevSpeed
  }
  avgRightWheelSpeed = 0;
  attachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin), RightWheelCount, FALLING);
  rightWheelInterrupt = 0;
}
void StopRightWheelSpeedControl()
{
  detachInterrupt(digitalPinToInterrupt(wheelSpeedRightPin));
}
void RightWheelCount()
{
  rightWheelInterrupt++;
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
  // Serial.print("left wheel rpm:");
  instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeOfLeftRev] = 60 * (float (deltaEncoder * 1000 / deltaTime) / leftWheelEncoderHoles);
  //  Serial.println(instantLeftWheelRevSpeed[leftWheelSpeedCount % sizeof(instantLeftWheelRevSpeed) / 2]);
  //  Serial.println(leftWheelSpeedCount % sizeOfLeftRev);
  leftWheelSpeedCount++;
}

void CheckMoveSynchronisation()
{
  float deltaMove = ((leftWheelInterrupt - rightWheelInterrupt) ) ;
  bitWrite(diagMotor, 3, 0);       // position bit diagMotor
  if ( abs(deltaMove) > 4)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    bitWrite(diagMotor, 3, 1);       // position bit diagMotor
    Serial.print("move synchro pb:");
    Serial.println(deltaMove);


  }
}
void ComputeNewLocalization(boolean last)
{
  float deltaLeft = ((float (leftWheelInterrupt) - saveLeftWheelInterrupt) * PI * iLeftWheelDiameter / leftWheelEncoderHoles);


  float deltaRight = ((float (rightWheelInterrupt) - saveRightWheelInterrupt) * PI * iRightWheelDiameter / rightWheelEncoderHoles);
  float deltaAlpha = 0;

  if ((deltaLeft != 0 && deltaRight != 0) || last == true)
  {
    Serial.print("newLoc:");
    Serial.print(leftWheelInterrupt);
    Serial.print(" ");
    Serial.print(saveLeftWheelInterrupt);
    Serial.print(" ");
    Serial.print(deltaLeft);
    Serial.print(" ");
    Serial.print(deltaRight);
    saveLeftWheelInterrupt = leftWheelInterrupt;
    saveRightWheelInterrupt = rightWheelInterrupt;
    float deltaC = (deltaLeft + deltaRight) / 2;
    if (deltaRight != 0)
    {
      deltaAlpha = (deltaLeft - deltaRight) / deltaRight;
    }
    else {
      deltaAlpha = deltaLeft / (PI * iLeftWheelDiameter);
    }
    alpha = deltaAlpha + alpha;
    Serial.print(deltaC);
    Serial.print(" ");
    Serial.println(alpha);
    deltaPosX = deltaPosX + deltaC * (1 - (deltaAlpha * deltaAlpha) / 2);
    deltaPosY = deltaPosY + deltaC * deltaAlpha;
    Serial.print("deltaPosX:");
    Serial.print(deltaPosX);
    Serial.print(" deltaPosY:");
    Serial.println(deltaPosY);
  }
  if (last == true)
  {
    posX = posX + deltaPosX;
    posY = posY + deltaPosY;
    Serial.print("new pox X:");
    Serial.print(posX);
    Serial.print(" Y:");
    Serial.print(posY);
    Serial.print(" angle:");
    Serial.println(alpha);
  }
}
void stopWheelControl()
{
  boolean last = true;
  PrintSpeed();
  ComputeNewLocalization(last);
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


void EchoServoAlign()
{
  myservo.attach(servoPin);
  myservo.write(pulseValue[(nbPulse + 1) / 2]);
  delay(1000);
  // myservo.detach();
}

void checkObstacle()
{
  int dist = 0;
  myservo.attach(servoPin);
  if (moveFront)
  {
    dist = PingFront() - 30;
  }
  else
    dist = PingBack();
  Serial.println(dist);
  if (dist < 50)
  {
    leftMotor.StopMotor();
    rightMotor.StopMotor();
    bitWrite(diagMotor, 7, 1);       // position bit diagMotor
    Serial.println("stop obstacle");
    delayToStopWheel = millis();
    bitWrite(pendingAction, pendingLeftMotor, false);
    bitWrite(pendingAction, pendingRightMotor, false);
  }

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
      Serial.println(diagMotor, HEX);
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

  if (deltaPosX >= 0)
  {
    PendingDataReqSerial[14] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[14] = 0x2d;
  }
  PendingDataReqSerial[15] = uint8_t(abs(deltaPosX) / 256);
  PendingDataReqSerial[16] = uint8_t(abs(deltaPosX));
  if (deltaPosY >= 0)
  {
    PendingDataReqSerial[17] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[17] = 0x2d;
  }
  PendingDataReqSerial[18] = uint8_t(abs(deltaPosY) / 256);
  PendingDataReqSerial[19] = uint8_t(abs(deltaPosY));

  if (alpha >= 0)
  {
    PendingDataReqSerial[20] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[20] = 0x2d;
  }
  int angle = alpha;
  Serial.println(angle);
  PendingDataReqSerial[21] = uint8_t(abs(angle) / 256);
  PendingDataReqSerial[22] = uint8_t(abs(angle));
  PendingDataLenSerial = 0x17; // 6 longueur mini pour la gateway
}
void SendPowerValue()
{
  Serial.print("power1:");
  power1Mesurt = 3 * map(analogRead(power1Value), 0, 1023, 0, 467);
  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 12v
  // Serial.println("cVolt");
  Serial.print(" power2:");
  if (power1Mesurt < 950)
  {
    bitWrite(diagPower, 0, 1);
  }
  else
  {
    bitWrite(diagPower, 0, 0);
  }
  power2Mesurt = 2 * map(analogRead(power2Value), 0, 1023, 0, 456);
  Serial.print(power2Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print("cV ");
  if (power2Mesurt < 470)
  {
    bitWrite(diagPower, 1, 1);
  }
  else
  {
    bitWrite(diagPower, 1, 0);
  }
  power3Mesurt = 2 * map(analogRead(power3Value), 0, 1023, 0, 540);
  if (power3Mesurt < 850)
  {
    bitWrite(diagPower, 2, 1);
  }
  else
  {
    bitWrite(diagPower, 2, 0);
  }
  Serial.print( "power3:");
  Serial.print(power3Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.println("cV");
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
}









































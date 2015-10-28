String Version = "RobotV0-2";
// passage sur GitHub
// test maj GitHub
// modif 16:27
// modif 16:33
// v25 et v26 mise au point liaison serie
// v24 remplacement RF433 par liaison serie vers rapsberry
// v22 modif pulsevalue
// v21 modif servo.h
// V 20 portage RF433 sans polling +ATmega
// v1-73 debug pour IDE 1.6.5
// v1-72 datain portee de 20 a 25
// test le 22/07 inversion pin 4 et 9
// 1.7 utilisation du service 15
// modifie demarrage  vw_set_ptt_pin(4);
// try servo calibrate > a tunner - demarrage en statut appli stopped
// conduit a la fonction y = 2,31 x +151 - y feedback (out tension mesuree sur A0) value x angle en degre
// ou x=(y-151)/2.31
// et x=(z/3.91-151)/2.31  z etant la valeur passe (ang) en input au servo
// entree analogique 0 sur
// alim stepper 5V - alim servo moteur 4,8V
// modif por stepper via UNL 2 moteurs
// modif port UDP demande statut + appel appel statut apres maj config
// modif statut et statistique pour conformite DB SQL
// integration de 2 detecteur ultrason - 1 moteur pas a pas 1 servo moteur
// v1-6 test 2 ultrason avec trigger en commun
// V1-5 stat passees en unsigned speed à 1200 - ajout demande de stat par le server
// V1-4 lecture en eeprom de l ID
// V1-3 ajout commande statut qui remonte tous les paramtres
// V1-3 simplification traitement des commandes en coherence avec MasterRF434V08-3
// 1-1 devlpt en cours pour reception de commande en provenance UDP en coherence avec MasterRF434V08-1";
// valeur data 0xff interdite a l envoi
// modif la longueur par 29 et non 58
// modif longueur sur 2 octets 1er * 256 + reste
// modif timeout de pulsin a 12000 - correspond a la longueur maxi de 4m
// sur base SlaveRF434V07-2"
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
int leftMotorIN2 = 4; //Connecté à Arduino pin 4

//-- right Motor connection --
int rightMotorENB = 6; //Connecté à Arduino pin 6(Sortie pwm)
int rightMotorIN3 = 5; //Connecté à Arduino pin 4
int rightMotorIN4 = 7; //Connecté à Arduino pin 7

//boolean bClockwise = true; //Used to turn the motor clockwise or counterclockwise
boolean bForward = true; //Used to drive traction chain forward
boolean bLeftClockwise = !bForward; //Need to turn counter-clockwise on left motor to get forward
boolean bRightClockwise = bForward; //Need to turn clockwise on left motor to get forward
//
unsigned long iDistance = 1000; // mm
int iSpeed = 409; // mm/s 408.4 maxi
unsigned long iLeftCentiRevolutions;
unsigned long iRightCentiRevolutions;
unsigned long restartedDelay;
int iLeftRevSpeed;
int iRightRevSpeed;

Motor leftMotor(leftMotorENA, leftMotorIN1, leftMotorIN2, iLeftMotorMaxrpm);
Motor rightMotor(rightMotorENB, rightMotorIN3, rightMotorIN4, iRightMotorMaxrpm);
//
unsigned long timeAppli; // cycle applicatif
unsigned long timeStatut;  // cycle envoi du statut au master
unsigned long timeNetwStat; // cycle affichage des stat RF433 sur serial
unsigned long timeRTC;  // cycle demande heure
unsigned long timeScanFront;  // cycle demande heure
unsigned long timeScanBack;  // cycle demande heure
unsigned long timeSendSec;  //
unsigned long timeSendSecSerial;  //
unsigned long timePower1Check;  //
unsigned long serialAliveTimer;
unsigned long serialTimer;
unsigned long durationMaxEcho = 25000; // en us
uint8_t Diag = 0xFF;
uint8_t toDo = 0x00; //
#define toDoScan 0
#define toDoMove 1
#define delayBetween2Scan 1500 // pour laisser le temps de positionner le servo et remonter data
#define delayBetweenScanFB 700  // delai entre 2 pulses
uint8_t SavDiag = 0xFF;
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

/* Utilisation du capteur Ultrason HC-SR04 */
//
// définition des broches utilisées
#define echoFront 22
#define trigF  23
#define echoBack  24
#define trigB  25
#define delayPowerCheck 60000
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
int minAng = 950; // testé: min absolu 950 et maxi 2100 - pour un angle de 132°
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
#define power1Value A15
#define power2Value A14
#define power3Value A13
int power1Mesurt = 0;
int power2Mesurt = 0;
int power3Mesurt = 0;
int nbPas = 2048;
//int pulseValue[16]={
// 981,1078,1133,1220,1287,1386,1442,1523,1598,1673,1748,1823,1898,1973,2048}; // liste des pulses stables
/*
int pulseValue[14] = {
  1078, 1133, 1220, 1287, 1386, 1442, 1523, 1598, 1673, 1748, 1823, 1898, 1973, 2048
}; // liste des pulses stables
*/
//int pulseValue[15] = {850, 943, 1036, 1129, 1221, 1292, 1442, 1500, 1593, 1686, 1779, 1871, 1964, 2057, 2150};
//int pulseValue[15] = {1299, 1290, 1291, 1292, 1293, 1294,1295, 1296, 1297, 1298, 1299, 1301, 1301,1302,1303};
//int pulseValue[15] = {850, 850, 850, 850, 1500, 1500,1500, 1500, 1500, 1500, 1500, 2150, 2150,2150,2150};
//int pulseValue[15] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
//float coefAngRef = 0.00177491;
float coefAngRef;
//float coefAngRead = 0.0064;
//int pulseValue[15] = {910, 994, 1079, 1163, 1247, 1331, 1416, 1500, 1584, 1669, 1753, 1837, 1921, 2006, 2090};
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
int rayonRobot = 150; // en mn
int rayonRoue = 37; //en mn
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

  //  vw_set_tx_pin(3);   // choisir la broche 3 pour émettre
  //  vw_set_rx_pin(5);     // choisir la broche 5 pour recevoir
  //  vw_set_ptt_pin(4);     // choisir la broche 9 pour ptt - utilite a confirmer
  //  vw_setup(SpeedNetw);       // vitesse de reception  - tunned pour etre en synch avec le master ??
  //  vw_rx_start();        // démarrer la réception
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
  //pinMode(power1Value, INPUT);

  digitalWrite(power1Pin, LOW);
  //  stepperD.setSpeed(172); // vitesse de bon fonctionnement 172
  //  stepperG.setSpeed(172); //
  Serial.print("nbpas: ");
  Serial.println(nbPas);
  coefAngRef = PI / (pulseValue[14] - pulseValue[0]);
  myservo.attach(servoPin);
  myservo.write(pulseValue[(nbPulse + 1) / 2]);
  delay(1000);
  myservo.detach();
  //  Serial.print ("free ram");
  // Serial.println(freeRam());
  ComputerMotorsRevolutionsAndrpm(iDistance, iSpeed, iDistance, iSpeed);
}

void loop() {
  int iLeftRpm = leftMotor.CheckMotor();
  int iRightRpm = rightMotor.CheckMotor();
  int getSerial = Serial_have_message();
  if (getSerial > 0)
  {
    //    Serial.print("getSerial:");

    //   Serial.println (getSerial);
    for (int i = 0; i < getSerial; i++)
    {
      //     Serial.print(DataInSerial[i],HEX);
      //    Serial.print("-");

    }
    //   Serial.println();
    TraitInput(DataInSerial[2]);
  }


  if (PendingReqSerial != 0x00)
  {
    //  Serial.println("datatosend");
    DataToSendSerial(); // bidouille pour forcer la gateway serial udp a lire le cote udp
  }

  else
  {
    ;
    /*
    if (millis() - serialAliveTimer >= 2000)
    {
      serialAliveTimer = millis();
      PendingDataReqSerial[0] = 0xff;

      PendingDataLenSerial = 0x01;
      //   pendingAckSerial = 0x01;
      PendingReqSerial = PendingReqRefSerial;
      DataToSendSerial();

    }
    */
  }


  if ((millis() - timeRTC) >= (235000 + random(0, 10000))) // toutes les 4mn (un cycle rate tous les 50 jours)
  {
    timeRTC = millis();
    nbRTC = nbRTC + 1; // compte nb demandes pending
    if (nbRTC > 5) {
      bitWrite(Diag, 4, 1);       // position bit diag
    }
    //   RequestTimeFromUDP();  // demande l heure au serveur
  }
  if ((millis() - timeStatut) >= (295000 + random(0, 10000))) // toutes les 5mn  (un cycle rate tous les 50 jours)
  {
    timeStatut = millis(); // au pas n° 5000 tous les 1000
    //   DemandStatus();   // remontee cycle du statut vers serveur
  }
  if ((millis() - timePower1Check) >= delayPowerCheck ) // toutes les 5mn  (un cycle rate tous les 50 jours)
  {
    PowerCheck();

  }
  /*
    cycleRetrySendUnit = cycleRetrySendUnit + 1;
    if ( cycleRetrySendUnit >= 30) {
      cycleRetrySendUnit = 1;
      uint8_t retCode = CheckUnitInd();
      if (retCode == 0x00)
      {
        // verifie que la derniere data a remonter de façon securisee a ete acquittee
      }
    }
    */
  // ci-dessous mettre le code applicatif
  if (bitRead(toDo, toDoScan) == 1)
  {

    ScanPosition();

  }
  bcl2 = bcl2 + 1;
  if (bcl2 > Lbcl2) {
    bcl2 = 0;
    /*  if (AppStat<0xff) {
     if (AppStat<0xe0) {  // a partir de 0xe0 plus de rotation
     if (AppStat==0xd0) {
     AppStat=0xe0;         // force position a fixe pour rotation pas a pas
     }
     //     val = IncPulse( myservo.read(), 50);

     //valAng=valR;
     }
     */
    // Serial.print("cycle appstat:");
    //  Serial.println(AppStat,HEX);
    // Serial.println(AppStat&0x0f,HEX);
    // Serial.println(AppStat&0xf0,HEX);

    if ((AppStat & 0x0f) < 0x0e) {
      //      Serial.println("scan pos");

    }

    if ((AppStat & 0xf0) < 0xe0) {
      Serial.println("move");
      MoveToward(reqAng, reqMove);
      AppStat = AppStat | 0xf0;
    }
    //    ScanPosition(1,0);
    //    MoveToward(-10,-300);
    //   }

    /*      for (int i=1;i<=nbPas;i++){
     //  Serial.print(i);
     stepperD.step(1);  // 1 pas
     stepperG.step(1);  //
     }
     Serial.print("pos:" );
     Serial.print(valAng);
     Serial.print(" pin: ");
     ServoFeedbackValue=analogRead(ServoFeedbackPin);
     Serial.println(ServoFeedbackValue);
     */
  }
  if (retryCount >= 3)
  {
    pendingAckSerial = 0x00;
    retryCount = 0;
  }
  if ( millis() - timeSendSecSerial >= 5000 && pendingAckSerial != 0x00 && retryCount < 3) {
    //   uint8_t retCode = (CheckUnitInd());
    ReSendSecuSerial();
    Serial.println("retry");
    timeSendSecSerial = millis();
    retryCount = retryCount + 1;
  }

}

void TraitInput(uint8_t cmdInput) {
  Serial.print("input:");
  Serial.println(cmdInput, HEX);
  switch (cmdInput) {
    case 0x73: // commande s
      Serial.println("cmd stop");
      AppStat = 0xff;
      break;
    case 0x78: // commande x
      Serial.println("cmd start");
      AppStat = 0x00;
      break;
    case 0x65: // commande e
      //     Serial.println(" echo");
      PendingReqSerial = PendingReqRefSerial;
      PendingDataReqSerial[0] = 0x65; //
      PendingDataReqSerial[1] = AppStat; //
      PendingDataReqSerial[2] = actStat;
      PendingDataReqSerial[3] = 0x00;
      PendingDataReqSerial[4] = 0x00;
      PendingDataReqSerial[5] = 0x00;
      PendingDataLenSerial = 0x06; // 6 longueur mini pour la gateway

      //    SendRFNoSecured();
      break;
    case 0x41: // commande A

      break;
    case 0x42: // commande B

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

      //     SendRFNoSecured();
      for (int i = 0; i <= 5; i++) {
        Serial.print(DataInSerial[i], HEX);
        Serial.print("-");
      }
      Serial.println("Move: ");
      bitWrite(toDo, toDoMove, 1);       // position bit toDo move
      reqAng = DataInSerial[4] * 256 + DataInSerial[5];
      if (DataInSerial[3] == 0x2d) {
        reqAng = -reqAng;
      }
      reqMove = DataInSerial[7] * 256 + DataInSerial[8];
      if (DataInSerial[6] == 0x2d) {
        reqMove = -reqMove;
      }
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
      //      Serial.print(DataInSerial[3], HEX);
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
      else
      {
        //      PendingDataReq[0] = 0x01;
        //      PendingSecReq =
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
  //  Serial.print("valAng:");
  //  Serial.println (valAng);
  myservo.write(valAng);
  /*
  if (vw_have_message())
   {
   TrameAnalyzeSlave();
   }
   */
  delay(1000);
}
int IncPulse(int sens) { // calcul du pulse
  //  Serial.print("sens:");
  //  Serial.println(sens);
  pulseNumber = pulseNumber + sens;
  return (pulseValue[(pulseNumber) % nbPulse]) ;

}
int PingFront() {
  /*  for (int i=0;i>=sizeof(pulseValue);i++)
   {
   PosServo();
   */
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
  // Serial.print(lecture_echo);
  // Serial.print(" deltaT Front:");
  //  Serial.println(time2 - time1);

  //    cm = lecture_echo / 58 * 1.23 - 3; // 1.25 coeff correction mesuree en reel -8 cm ecart VS roues
  //    cm = lecture_echo / 58  ; // 1.25 coeff correction mesuree en reel -8 cm ecart VS roues
  cm = deltaT / 58  ;
  //Serial.println(cm);
  /*
    Serial.print("Front t1:");
    Serial.print(time1);
    Serial.print(" t2:");
    Serial.print(time2);
    Serial.print("dT:");
    Serial.println(deltaT);
  */
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




  /*
  Serial.print("Back t1:");
  Serial.print(time1);
  Serial.print(" t2:");
  Serial.print(time2);
  Serial.print("dT:");
  Serial.println(time2 - time1);
  */
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
  /*
    Serial.print("scan:");
    Serial.print(numStep);
    Serial.print("-");
    Serial.print(nbSteps);
   Serial.print("-");
    Serial.println(switchFB);
  */
  // if ((millis() - timeScanFront) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 )
  if ((millis() - timeScanBack) > delayBetween2Scan && numStep <= abs(nbSteps) - 1 && switchFB == 0 && pendingAckSerial == 0x00)
  {

    timeScanFront = millis();
    switchFB = 1;
    distFSav = ScanOnceFront(numStep);
    //      vw_rx_start();        // démarrer la réception
  }
  if ((millis() - timeScanFront) > delayBetweenScanFB && numStep <= abs(nbSteps) - 1 && switchFB == 1 && pendingAckSerial == 0x00)
    //  if ((millis() - timeScanBack) > delayBetweenScanFB && numStep <= abs(nbSteps) - 1 && switchFB == 1 )
  {
    timeScanBack = millis();
    switchFB = 0;
    distBSav = ScanOnceBack(numStep);
    //          vw_rx_start();        // démarrer la réception

    //     PosServo(abs(nbSteps)/nbSteps);
    // -1 ou +1

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
  //  bool checkFeeback = CheckFeedback();
  //  servoFeedbackValue = analogRead(servoFeedbackPin);
  // if ((abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue))) / servoFeedbackValue > 1.1)
  // if ((abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue)))  >= 1.05)
  // {
  //  Serial.print("pb feedback:");
  //  Serial.print(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue) / abs(pulseFeedback[(pulseNumber) % nbPulse]));
  // }
  // Serial.println();
  //  Serial.print("AngleF:");
  // Serial.print(servoFeedbackValue);
  //      AngleRadian = ((servoFeedbackValue- angleD) * coefAngRead);
  // AngleRadian = ((servoFeedbackValue - 158) / 128);
  AngleRadian = (pulseValue[(pulseNumber) % nbPulse] - pulseValue[0]) * coefAngRef;
  AngleDegre = (AngleRadian / PI) * 180;
  // Serial.print(AngleRadian);
  //  Serial.print(";");
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
  //  servoFeedbackValue = analogRead(servoFeedbackPin);
  //  bool checkFeeback = CheckFeedback();
  /*
    if ((abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue)))  >= 1.05)
    {
      Serial.print("pb feedback:");
      Serial.print(abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue)) / servoFeedbackValue);
    }
    */
  //  Serial.println();
  //  Serial.print("AngleB:");
  //  Serial.print(servoFeedbackValue);
  //  AngleRadian = ((servoFeedbackValue - 158) / 128);
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
  // DataToSendSerial();
  //  timeSendSecSerial = millis();
  myservo.write(pulseValue[(nbPulse + 1) / 2]); // remise au centre
  delay(1000);
  myservo.detach();  // attaches the servo on pin 4 to the servo object
  digitalWrite(power1Pin, LOW);
  numStep = 0;
  // SendRFNoSecured();
}
void MoveToward(int orientation, int length) {
  //  int nbPasToDo[4]={
  //   MoveTowardCalculation(orientation,length)   };
  int s1;
  float* revolutionsToDo = MoveTowardCalculation(orientation, length) ;
  for (int i = 0; i <= 3; i++)
  {
    Serial.println(revolutionsToDo[i]);
  }
  unsigned long lLeftCentiRevolutions = 100 * abs(revolutionsToDo[0]);
  unsigned long lRightCentiRevolutions = 100 * abs(revolutionsToDo[1]);
  boolean LeftClockwise;
  boolean RightClockwise;
  // rotation 
  if (revolutionsToDo[0] < 0)
  {
    LeftClockwise = bLeftClockwise * -1;
  }
  else
  {
    LeftClockwise = bLeftClockwise;
  }
  if (revolutionsToDo[1] < 0)
  {
    RightClockwise = RightClockwise * -1;
  }
  else
  {
    RightClockwise = bRightClockwise;
  }
  lLeftCentiRevolutions = 100 * revolutionsToDo[0];
  lRightCentiRevolutions = 100 * revolutionsToDo[1];
  leftMotor.TurnMotor(bLeftClockwise, lLeftCentiRevolutions, iLeftRevSpeed);
  rightMotor.TurnMotor(bRightClockwise, lRightCentiRevolutions, iRightRevSpeed);
  //
  // mouvement rectiligne
  if (revolutionsToDo[2] < 0)
  {
    LeftClockwise = bLeftClockwise * -1;
  }
  else
  {
    LeftClockwise = bLeftClockwise;
  }
  if (revolutionsToDo[3] < 0)
  {
    RightClockwise = RightClockwise * -1;
  }
  else
  {
    RightClockwise = bRightClockwise;
  }
  lLeftCentiRevolutions = 100 * revolutionsToDo[2];
  lRightCentiRevolutions = 100 * revolutionsToDo[3];
  leftMotor.TurnMotor(bLeftClockwise, lLeftCentiRevolutions, iLeftRevSpeed);
  rightMotor.TurnMotor(bRightClockwise, lRightCentiRevolutions, iRightRevSpeed);
  //  if (orientation != 0) {
  //    s1 = (orientation < 0) ? -1 : +1;
  //  }
  //  float distCorrige = abs(distToDo[0]) * coeffGlissementRotation;

  // Serial.print("nbPasCorrige:");
  // Serial.println(nbPasCorrige);
  /*
  int j = 0;
  for (int i = 1; i <= (nbPasCorrige); i++) {
    if  ((AppStat & 0xf0) < 0xe0) {
  //      stepperD.step(s1);  //
  //      stepperG.step(s1);  //
    }
    j = j + 1;
    if (j % 72 == 0) {

    }
  }
  delay(200);
  if (length != 0) {
    s1 = (length < 0) ? -1 : +1;
  }
  //  j=0;
  for (int i = 1; i <= abs(nbPasToDo[2]); i++) {
    if  ((AppStat & 0xf0) < 0xe0) {
  //      stepperD.step(s1);  //
  //      stepperG.step(-s1);  //
    }
    j = j + 1;
    if (j % 72 == 0) {
      //      if (vw_have_message())
      //     {
      //       TrameAnalyzeSlave();
      //     }
    }

  }
  */
  Serial.println("move ok");
  actStat = 0x69; //" move completed

  // SendRFNoSecured();
}


float*  MoveTowardCalculation(int orientation, int length) {
  float *revolutionsToDo = (float *) malloc(sizeof(float) * 4);
  int nbPasG1;
  int nbPasD2;
  int nbPasG2;
  Serial.print("sinus:");
  float angleRadian = orientation;
  angleRadian = angleRadian / 180 * PI;
  Serial.println(angleRadian);
  Serial.println(sin(angleRadian));
  Serial.println(rayonRobot * sin(angleRadian));
  revolutionsToDo[0] = ((rayonRobot * PI / 180) * orientation);
  revolutionsToDo[1] = -revolutionsToDo[0];
  if (orientation < 0)
  {
    revolutionsToDo[0] = - revolutionsToDo[0];
    revolutionsToDo[1] = - revolutionsToDo[1] ;
  }
  //  nbPasToDo[1]=rayonRobot*sin(orientation)*nbPas/(2*PI*rayonRoue);
  //  nbPasG1=nbPasD1[0];
  revolutionsToDo[2] = (length / (2 * PI * rayonRoue));
  revolutionsToDo[3] = (length / (2 * PI * rayonRoue));
  //  nbPasG2=-nbPasD2;
  Serial.print("tours D1:");
  Serial.println(revolutionsToDo[0]);
  Serial.print("tours D2:");
  Serial.println(revolutionsToDo[2]);
  return (revolutionsToDo);
}

bool CheckFeedback()
{
  Serial.print("Checkfeedback:");
  Serial.print(abs(pulseValue[(pulseNumber) % nbPulse] ));
  servoFeedbackValue = analogRead(servoFeedbackPin);
  Serial.print(" ");
  Serial.println(servoFeedbackValue);
  // if ((abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue))) / servoFeedbackValue > 1.1)
  // if ((abs(abs(pulseFeedback[(pulseNumber) % nbPulse] - servoFeedbackValue)))  >= 1.05)
  // {
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
/* exemple appel de la remontee data securisee
 PendingReq=0x32; // routage UDP
 PendingDataReq[1]=50+CptSch;  // n° indicateur
 PendingDataReq[2]=Schedul[CptSch];
 UnitInd();
 */

void SendScanResultSerial (int distF, int distB)
{
  retryCount = 00;
  PendingDataReqSerial[0] = 0x01;
  PendingDataReqSerial[1] = 0x46; //"B" front
  PendingDataReqSerial[2] = uint8_t(distF / 256);
  PendingDataReqSerial[3] = uint8_t(distF);
  PendingDataReqSerial[4] = 0x42; //"B" back
  PendingDataReqSerial[5] = uint8_t(distB / 256);
  PendingDataReqSerial[6] = uint8_t(distB);
  PendingDataReqSerial[7] = uint8_t(AngleDegre / 256); //1er octet contient les facteurs de 256
  PendingDataReqSerial[8] = uint8_t(AngleDegre); //2eme octets contient le complement - position = Datareq2*256+Datareq3
  PendingDataReqSerial[9] = uint8_t(trameNumber % 256);
  PendingDataLenSerial = 0x0a;
  pendingAckSerial = 0x01;
  /*
    for (int i=0;i<10;i++)
    {
      Serial.print(PendingDataReqSerial[i],HEX);
      Serial.print("-");
    }
     Serial.println();
  */
  PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  SendSecuSerial();
  timeSendSecSerial = millis();
}



void PowerCheck()
{
  timePower1Check = millis(); //
  Serial.print("power1:");
  power1Mesurt = 3 * map(analogRead(power1Value), 0, 1023, 0, 467);
  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 12v
  Serial.println("cv");
  Serial.print("power2:");
  power2Mesurt = 2 * map(analogRead(power2Value), 0, 1023, 0, 456);
  Serial.print(power2Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.println("cv");
  power3Mesurt = 2 * map(analogRead(power3Value), 0, 1023, 0, 540);
  Serial.print("power3:");
  Serial.print(power3Mesurt); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.println("cv");
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x70; //
  PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  PendingDataReqSerial[3] = uint8_t(power3Mesurt / 10);
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = 0x00;
  PendingDataLenSerial = 0x06; // 6 longueur mini pour la gateway

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










































































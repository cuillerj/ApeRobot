void IncServo(int sens, int waitDuration) {  // increase servo motor position depending on sens value
  valAng = IncPulse(sens ); // compute servo motor value depending on sens value
  myservo.attach(servoPin);
  //  delay(100);
#if(servoMotorDebugOn)
  Serial.print("servo write pulse:");
  Serial.println(valAng);
#endif
  myservo.write(valAng + shiftPulse);  // move servo motor
  delay(waitDuration);              // wait to be sure the servo  get the order
  myservo.detach();
  delay(100);                      // wait to be sure the servo reach the position
}
int IncPulse(int sens) { // compute servo motor value depending on sens
  pulseNumber = pulseNumber + sens;    //
  return (pulseValue[(pulseNumber) % nbPulse]) ;
}
int PingFront() {               // ping echo front
  float cm;
  unsigned int uS = pingFront.ping_median(3); // Send ping, get ping time in microseconds (uS).
  if (uS == 0)
  {
#if (debugScanOn)
    Serial.println("retry front");
#endif
    delay(70);
    uS = pingFront.ping();
  }
  cm = pingFront.convert_cm(uS);
  if (cm != 0)                  // keep 0 value (no mesurment)
  {
    cm = cm + shiftEchoFrontBack / 2;   // add  to adjust center location
  }
#if (debugScanOn)
  Serial.print("Ping front: ");
  Serial.print(cm); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
#endif
  return (cm);
}

int PingBack() {
  float cm;

  unsigned int uS = pingBack.ping(); // Send ping, get ping time in microseconds (uS).
  // Serial.print("Ping: ");


  if (uS == 0)
  {
#if (debugScanOn)
    Serial.println("retry back");
#endif
    delay(70);
    uS = pingFront.ping();
  }
  cm = pingBack.convert_cm(uS) ;
  if (cm != 0)
  {
    cm = cm + shiftEchoFrontBack / 2;
  }
#if (debugScanOn)
  Serial.print("Ping back: ");
  Serial.print(cm); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
#endif
  return (cm);

  //  }
}

void PingFrontBack()
{
  int distFront = PingFront();                          // added 28/09/2016
  delay(100);                                            // added 24/10/2016
  int distBack = PingBack();
  SendScanResultSerial (distFront, distBack);
  timePingFB = millis();
}
void InitScan(int nbS, int startingOrientation)    // int scan
{
  numStep = 0;                                     // init current number step
  nbSteps = nbS;                                   // init number of servo steps to do
  scanOrientation = startingOrientation;           // init starting servo position
  myservo.attach(servoPin);                       // attaches the servo motor
  //  delay(100);
#if (servoMotorDebugOn)
  Serial.print("init pulse:");
  Serial.println(pulseValue[scanOrientation]);
#endif
  myservo.write(pulseValue[scanOrientation] + shiftPulse);   // set the servo motor to the starting position
  delay(1000);                                    // wait long enough for the servo to reach target
  myservo.detach();
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
    if (numStep < abs(nbSteps) - 1)
    {
      IncServo(1, 200);
    }
    numStep = numStep + 1;
  }

  if (pendingAckSerial == 0x00 && numStep >= abs(nbSteps)  )
  {
    Serial.println("scanEnd");
    bitWrite(toDo, toDoScan, 0);       // position bit toDo scan
    timeScanBack = millis();
    switchFB = 0;
    EchoServoAlign(servoAlignedPosition,false);
    //    myservo.write(pulseValue[(nbPulse + 1) / 2]); // remise au centre
    //   delay(1000);
    //   myservo.detach();
    SetBNOMode(MODE_IMUPLUS);
    SendEndAction(scanEnded, 0x00);
  }
}

int ScanOnceFront(int numStep)
{
  int distF = PingFront();

  //  AngleRadian = (pulseValue[(pulseNumber) % nbPulse] - pulseValue[0]) * coefAngRef;
  //  AngleDegre = (AngleRadian / PI) * 180;
  AngleDegre = (pulseValue[(pulseNumber) % nbPulse]);
#if defined(debugScanOn)
  Serial.print(" ");
  Serial.print(AngleDegre);
  Serial.print(" dist front:");
  Serial.println(distF);
#endif
  return distF;

}
int ScanOnceBack(int numStep)
{
  int distB = PingBack();
  trameNumber = trameNumber + 1;
  // AngleRadian = (pulseValue[(pulseNumber) % nbPulse] - pulseValue[0]) * coefAngRef;
  //  AngleDegre = AngleRadian / PI * 180;
  AngleDegre = (pulseValue[(pulseNumber) % nbPulse]);
  // Serial.print(AngleRadian);
  //  Serial.print(";");
#if defined(debugScanOn)
  Serial.print(" ");
  Serial.print(AngleDegre);
  Serial.print(" dist back:");
  Serial.print(distB);
  Serial.println();
#endif
  return distB;

}
void EchoServoAlign(uint8_t angle, boolean report)   // to align echo servo motor with robot
{
  AngleDegre = max(miniServoAngle,min(angle,maxiServoAngle));
  myservo.attach(servoPin);
  delay(5);
  // unsigned int value = map(angle, 0, 180, pulseValue[0],  pulseValue[nbPulse - 1]);

#if(debugScanOn)
  Serial.print("servo align:");
  //  Serial.println(pulseValue[(nbPulse - 1) / 2]);
  Serial.println(AngleDegre);
#endif
  //  myservo.write(pulseValue[(nbPulse - 1) / 2]);  // select the middle of the pulse range
  myservo.write(AngleDegre + shiftPulse);
  delay(750);
  myservo.detach();
  delay(10);
  if (report)
  {
    SendEndAction(servoAlignEnded, 0x00);
  }

}

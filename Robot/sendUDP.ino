void SendEndAction(int action, uint8_t retCode)
{
#if defined(debugGyroscopeOn)
  Serial.print("gyroscope data:");
  for (int i = 0; i < gyroscopeHeadingIdx; i++)
  {
    Serial.print(gyroscopeHeading[i]);
    Serial.print(" ");
  }
  Serial.println();

#endif
  //  GetHeadingRegisters();
  int northOrientation = NorthOrientation();
  int deltaNORotation = NOBeforeRotation - NOAfterRotation;
  int deltaNOMoving = NOBeforeMoving - NOAfterMoving;
  actStat = action;
  trameNumber = trameNumber + 1;
  PendingDataReqSerial[0] = 0x01;   // ack expected
  PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  PendingDataReqSerial[2] = 0x01;
  PendingDataReqSerial[3] = actStat;
  PendingDataReqSerial[4] = retCode;

  PendingDataReqSerial[5] = 0x00;
  PendingDataReqSerial[6] = uint8_t(northOrientation / 256);
  PendingDataReqSerial[7] = uint8_t(northOrientation);

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
  if (deltaNORotation >= 0)
  {
    PendingDataReqSerial[17] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[17] = 0x2d;
  }
  PendingDataReqSerial[18] = uint8_t(abs(deltaNORotation) / 256);
  PendingDataReqSerial[19] = uint8_t(abs(deltaNORotation));

  if (gyroscopeHeading[gyroscopeHeadingIdx] >= 0)
  {
    PendingDataReqSerial[20] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[20] = 0x2d;
  }
  PendingDataReqSerial[21] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]) / 256);
  PendingDataReqSerial[22] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]));
  if (deltaNOMoving >= 0)
  {
    PendingDataReqSerial[20] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[20] = 0x2d;
  }
  PendingDataReqSerial[21] = uint8_t(abs(deltaNOMoving) / 256);
  PendingDataReqSerial[22] = uint8_t(abs(deltaNOMoving));

  // below dependin on actStat
  PendingDataReqSerial[23] = 0x2f;
  PendingDataReqSerial[24] = 0x00;
  PendingDataReqSerial[25] = 0x00;
  PendingDataReqSerial[26] = 0x00;
  PendingDataReqSerial[27] = 0x00;
  PendingDataReqSerial[28] = 0x00;
  PendingDataReqSerial[29] = 0x00;
  if (gyroscopeHeading[gyroscopeHeadingIdx] >= 0)
  {
    PendingDataReqSerial[23] = 0x2b;
  }
  else
  {
    PendingDataReqSerial[23] = 0x2d;
  }
  PendingDataReqSerial[24] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]) / 256);
  PendingDataReqSerial[25] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]));
  /* if (actStat == moveEnded)
    {
     PendingDataReqSerial[24] = uint8_t(leftWheeelCumulative / 256);
     PendingDataReqSerial[25] = uint8_t(leftWheeelCumulative);
     PendingDataReqSerial[26] = 0x00;
     PendingDataReqSerial[27] = uint8_t(rightWheeelCumulative / 256);
     PendingDataReqSerial[28] = uint8_t(rightWheeelCumulative);
    }
  */
  PendingDataLenSerial = 0x1e;
  retryCount = 00;
  pendingAckSerial = 0x01;
  PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();
  timeSendInfo = millis();
  // myservo.detach();  // attaches the servo on pin 4 to the servo object
  // digitalWrite(power1Pin, LOW);
  numStep = 0;
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
  bitWrite(monitSubsystemStatus, I2CConnectionBit, digitalRead(RobotInputReadyPin));
  PendingDataReqSerial[6] = monitSubsystemStatus;
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
//  northOrientation = saveNorthOrientation;
 // if (toDo == 0x00 && (actStat != 0x66 && actStat != 0x68) && millis() - delayAfterStopMotors > 500 )
 // {
 //   northOrientation = NorthOrientation();
 // }
  PendingDataReqSerial[18] = uint8_t(northOrientation / 256);
  PendingDataReqSerial[19] = uint8_t(northOrientation);
  PendingDataReqSerial[20] = 0x00;
  PendingDataReqSerial[21] = currentLocProb;
  PendingDataReqSerial[22] = 0x00;
  PendingDataReqSerial[23] = toDo;   // for debuging
  PendingDataReqSerial[24] = 0x00;
  PendingDataReqSerial[25] = pendingAction; // for debuging
  PendingDataReqSerial[26] = 0x00;
  PendingDataReqSerial[27] =  waitFlag; // for debuging
  PendingDataLenSerial = 0x1c; // 6 longueur mini max 25 pour la gateway
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
void SendEncoderMotorValue()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x71; //
  PendingDataReqSerial[1] = 0x07;
  PendingDataReqSerial[2] = uint8_t(leftIncoderHighValue / 256);
  PendingDataReqSerial[3] = uint8_t(leftIncoderHighValue );
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = uint8_t(leftIncoderLowValue / 256);
  PendingDataReqSerial[6] = uint8_t(leftIncoderLowValue );
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = uint8_t(rightIncoderHighValue / 256);
  PendingDataReqSerial[9] = uint8_t(rightIncoderHighValue );
  PendingDataReqSerial[10] = 0x00;
  PendingDataReqSerial[11] = uint8_t(rightIncoderLowValue / 256);
  PendingDataReqSerial[12] = uint8_t(rightIncoderLowValue );
  PendingDataReqSerial[13] = 0x00;
  PendingDataReqSerial[14] = uint8_t(leftMotorPWM / 256);
  PendingDataReqSerial[15] = uint8_t(leftMotorPWM );
  PendingDataReqSerial[16] = 0x00;
  PendingDataReqSerial[17] = uint8_t(rightMotorPWM / 256);
  PendingDataReqSerial[18] = uint8_t(rightMotorPWM );
  PendingDataReqSerial[19] = 0x00;
  PendingDataReqSerial[20] = uint8_t(leftToRightDynamicAdjustRatio * 100 / 256);
  PendingDataReqSerial[21] = uint8_t(leftToRightDynamicAdjustRatio * 100 );
  PendingDataLenSerial = 0x16; // 6 longueur mini max 25  pour la gateway
}
void SendEncoderValues()
{
  PendingReqSerial = PendingReqRefSerial;
  unsigned int minLeftLevel = Wheels.GetMinLevel(leftWheelId);
  unsigned int maxLeftLevel = Wheels.GetMaxLevel(leftWheelId);
  unsigned int minRightLevel = Wheels.GetMinLevel(rightWheelId);
  unsigned int maxRightLevel = Wheels.GetMaxLevel(rightWheelId);
  PendingDataReqSerial[0] = 0x72; //
  PendingDataReqSerial[1] = 0x08;   // paramters number
  PendingDataReqSerial[2] = uint8_t(leftIncoderHighValue / 256);
  PendingDataReqSerial[3] = uint8_t(leftIncoderHighValue );
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = uint8_t(maxLeftLevel / 256);
  PendingDataReqSerial[6] = uint8_t(maxLeftLevel );
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = uint8_t(leftIncoderLowValue / 256);
  PendingDataReqSerial[9] = uint8_t(leftIncoderLowValue );
  PendingDataReqSerial[10] = 0x00;
  PendingDataReqSerial[11] = uint8_t(minLeftLevel / 256);
  PendingDataReqSerial[12] = uint8_t(minLeftLevel );
  PendingDataReqSerial[13] = 0x00;
  PendingDataReqSerial[14] = uint8_t(rightIncoderHighValue / 256);
  PendingDataReqSerial[15] = uint8_t(rightIncoderHighValue );
  PendingDataReqSerial[16] = 0x00;
  PendingDataReqSerial[17] = uint8_t(maxRightLevel / 256);
  PendingDataReqSerial[18] = uint8_t(maxRightLevel );
  PendingDataReqSerial[19] = 0x00;
  PendingDataReqSerial[20] = uint8_t(rightIncoderLowValue / 256);
  PendingDataReqSerial[21] = uint8_t(rightIncoderLowValue );
  PendingDataReqSerial[22] = 0x00;
  PendingDataReqSerial[23] = uint8_t(minRightLevel / 256);
  PendingDataReqSerial[24] = uint8_t(minRightLevel );
  PendingDataLenSerial = 0x19; // 6 longueur mini max 25  pour la gateway
}
void SendPWMValues()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x73; //
  PendingDataReqSerial[1] = 0x02;   // paramters number
  PendingDataReqSerial[2] = uint8_t(leftMotorPWM / 256);
  PendingDataReqSerial[3] = uint8_t(leftMotorPWM );
  PendingDataReqSerial[4] = 0x00;
  PendingDataReqSerial[5] = uint8_t(rightMotorPWM / 256);
  PendingDataReqSerial[6] = uint8_t(rightMotorPWM );
  PendingDataReqSerial[7] = 0x00;
  PendingDataReqSerial[8] = uint8_t((leftRotatePWMRatio * 100) / 256);
  PendingDataReqSerial[9] = uint8_t(leftRotatePWMRatio * 100);
  PendingDataLenSerial = 0x0a; // 6 longueur mini max 25  pour la gateway
}
void SendEncodersHolesValues()
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x74; //
  PendingDataReqSerial[1] = 0x02;   // paramters number
  PendingDataReqSerial[2] = uint8_t(leftWheeelCumulative / 256);
  PendingDataReqSerial[3] = uint8_t(leftWheeelCumulative);
  PendingDataReqSerial[4] = 0x3a;
  PendingDataReqSerial[5] = uint8_t(rightWheeelCumulative / 256);
  PendingDataReqSerial[6] = uint8_t(rightWheeelCumulative);
  PendingDataLenSerial = 0x07; // 6 longueur mini max 25  pour la gateway
}
void  SendUDPSubsystemRegister(uint8_t receivedRegister[5], uint8_t receivedValue[5])
{
  PendingReqSerial = PendingReqRefSerial;
  PendingDataReqSerial[0] = 0x80; //
  PendingDataReqSerial[1] = 0x05;   // paramters number
  for (int i = 0; i < 5; i++)
  {
    PendingDataReqSerial[2 + 3 * i] = receivedRegister[i];
    PendingDataReqSerial[3 + 3 * i] = receivedValue[i];
    PendingDataReqSerial[4 + 3 * i] = 0x00;
  }
  PendingDataLenSerial = 0x12; // 6 longueur mini max 25  pour la gateway
}
void SendScanResultSerial (int distF, int distB)   // send scan echo data to the server
{

  PendingDataReqSerial[0] = 0x01;
  PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  PendingDataReqSerial[2] = 0x46; //"F" front
  PendingDataReqSerial[3] = uint8_t(distF / 256);
  PendingDataReqSerial[4] = uint8_t(distF);
  PendingDataReqSerial[5] = 0x42; //"B" back
  PendingDataReqSerial[6] = uint8_t(distB / 256);
  PendingDataReqSerial[7] = uint8_t(distB);
  PendingDataReqSerial[8] = 0x41; //"A" angle du servo
  PendingDataReqSerial[9] = uint8_t(AngleDegre / 256); //1er octet contient les facteurs de 256
  PendingDataReqSerial[10] = uint8_t(AngleDegre); //2eme octets contient le complement - position = Datareq2*256+Datareq3
  PendingDataReqSerial[11] = uint8_t(trameNumber % 256);
  PendingDataReqSerial[12] = 0x00;
  PendingDataReqSerial[13] = 0x00;
  PendingDataReqSerial[14] = 0x00;
  PendingDataLenSerial = 0x0f;                      // data len
  retryCount = 00;
  pendingAckSerial = 0x01;
  PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();                   // init timer used to check for server ack
}

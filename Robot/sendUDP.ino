uint8_t lastActionRetcode = 0x00;
void SendEndAction(uint8_t action, uint8_t retCode)
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
  lastActionRetcode = retCode;
  if (action == moveEnded || action == moveAcrossPassEnded)
  {
    GyroStopInitMonitor();
    digitalWrite(encoderPower, LOW);
  }
  sendInfoSwitch = 1;        // for sending status priority
  timeSendInfo = millis(); // for delaying the next automatic send
  //  GetHeadingRegisters();
  //  int northOrientation = NorthOrientation();
  int deltaNORotation = NOBeforeRotation - NOAfterRotation;
  int deltaNOMoving = NOBeforeMoving - NOAfterMoving;
  actStat = action;
  trameNumber = trameNumber + 1;
  GatewayLink.PendingDataReqSerial[0] = 0x01;   // ack expected
  GatewayLink.PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  GatewayLink.PendingDataReqSerial[2] = 0x01;
  GatewayLink.PendingDataReqSerial[3] = actStat;
  GatewayLink.PendingDataReqSerial[4] = retCode;

  GatewayLink.PendingDataReqSerial[5] = 0x00;
  if (action == requestUpdateNO)
  {
    GatewayLink.PendingDataReqSerial[6] = uint8_t(saveNorthOrientation / 256);
    GatewayLink.PendingDataReqSerial[7] = uint8_t(saveNorthOrientation);
  }
  else {
    GatewayLink.PendingDataReqSerial[6] = uint8_t(northOrientation / 256);
    GatewayLink.PendingDataReqSerial[7] = uint8_t(northOrientation);
  }


  if (posX >= 0)
  {
    GatewayLink.PendingDataReqSerial[8] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[8] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[9] = uint8_t(abs(posX) / 256);
  GatewayLink.PendingDataReqSerial[10] = uint8_t(abs(posX));
  if (posY >= 0)
  {
    GatewayLink.PendingDataReqSerial[11] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[11] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[12] = uint8_t(abs(posY) / 256);
  GatewayLink.PendingDataReqSerial[13] = uint8_t(abs(posY));

  if (alpha >= 0)
  {
    GatewayLink.PendingDataReqSerial[14] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[14] = 0x2d;
  }
  int angle = alpha;
  GatewayLink.PendingDataReqSerial[15] = uint8_t(abs(angle) / 256);
  GatewayLink.PendingDataReqSerial[16] = uint8_t(abs(angle));
  if (deltaNORotation >= 0)
  {
    GatewayLink.PendingDataReqSerial[17] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[17] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[18] = uint8_t(abs(deltaNORotation) / 256);
  GatewayLink.PendingDataReqSerial[19] = uint8_t(abs(deltaNORotation));

  if (gyroscopeHeading[gyroscopeHeadingIdx] >= 0)
  {
    GatewayLink.PendingDataReqSerial[20] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[20] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[21] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]) / 256);
  GatewayLink.PendingDataReqSerial[22] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]));
  if (deltaNOMoving >= 0)
  {
    GatewayLink.PendingDataReqSerial[20] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[20] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[21] = uint8_t(abs(deltaNOMoving) / 256);
  GatewayLink.PendingDataReqSerial[22] = uint8_t(abs(deltaNOMoving));

  // below depending on actStat
  GatewayLink.PendingDataReqSerial[23] = 0x2f;
  GatewayLink.PendingDataReqSerial[24] = 0x00;
  GatewayLink.PendingDataReqSerial[25] = 0x00;
  GatewayLink.PendingDataReqSerial[26] = 0x00;
  // some complementary information from 27 to 29
  if (action == moveEnded && retCode == moveKoDueToNotEnoughSpace)
  {
    if (movePingMax >= 0)
    {
      GatewayLink.PendingDataReqSerial[27] = 0x2b;
    }
    else {
      GatewayLink.PendingDataReqSerial[27] = 0x2d;
    }

    GatewayLink.PendingDataReqSerial[28] = uint8_t(abs(movePingMax) / 256);
    GatewayLink.PendingDataReqSerial[29] = uint8_t(abs(movePingMax));
  }
  else {
    GatewayLink.PendingDataReqSerial[27] = 0x00;
    GatewayLink.PendingDataReqSerial[28] = 0x00;
    GatewayLink.PendingDataReqSerial[29] = 0x00;
  }

  if (gyroscopeHeading[gyroscopeHeadingIdx] >= 0)
  {
    GatewayLink.PendingDataReqSerial[23] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[23] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[24] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]) / 256);
  GatewayLink.PendingDataReqSerial[25] = uint8_t(abs(gyroscopeHeading[gyroscopeHeadingIdx]));
  /* if (actStat == moveEnded)
    {
     GatewayLink.PendingDataReqSerial[24] = uint8_t(leftWheeelCumulative / 256);
     GatewayLink.PendingDataReqSerial[25] = uint8_t(leftWheeelCumulative);
     GatewayLink.PendingDataReqSerial[26] = 0x00;
     GatewayLink.PendingDataReqSerial[27] = uint8_t(rightWheeelCumulative / 256);
     GatewayLink.PendingDataReqSerial[28] = uint8_t(rightWheeelCumulative);
    }
  */
  GatewayLink.PendingDataLenSerial = 0x1e;
  retryCount = 00;
  pendingAckSerial = 0x01;
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  GatewayLink.SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();
  timeSendInfo = millis();
  // myservo.detach();  // attaches the servo on pin 4 to the servo object
  // digitalWrite(power1Pin, LOW);
  numStep = 0;
}
void SendStatus()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x65; //
  GatewayLink.PendingDataReqSerial[1] = appStat; //
  GatewayLink.PendingDataReqSerial[2] = actStat;
  GatewayLink.PendingDataReqSerial[3] = diagPower;
  GatewayLink.PendingDataReqSerial[4] = 0x00; // no more than 3 conscutives non asccii bytes
  GatewayLink.PendingDataReqSerial[5] = diagMotor;
  bitWrite(monitSubsystemStatus, I2CConnectionBit, digitalRead(RobotInputReadyPin));
  GatewayLink.PendingDataReqSerial[6] = monitSubsystemStatus;
  GatewayLink.PendingDataReqSerial[7] = diagRobot;
  if (posX >= 0)
  {
    GatewayLink.PendingDataReqSerial[8] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[8] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[9] = uint8_t(abs(posX) / 256);
  GatewayLink.PendingDataReqSerial[10] = uint8_t(abs(posX));
  if (posY >= 0)
  {
    GatewayLink.PendingDataReqSerial[11] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[11] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[12] = uint8_t(abs(posY) / 256);
  GatewayLink.PendingDataReqSerial[13] = uint8_t(abs(posY));

  if (alpha >= 0)
  {
    GatewayLink.PendingDataReqSerial[14] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[14] = 0x2d;
  }
  int angle = (int)alpha % 360;
  GatewayLink.PendingDataReqSerial[15] = uint8_t(abs(angle) / 256);
  GatewayLink.PendingDataReqSerial[16] = uint8_t(abs(angle));
  //  GatewayLink.PendingDataReqSerial[17] = BNOMode;
  //  northOrientation = saveNorthOrientation;
  // if (toDo == 0x00 && (actStat != 0x66 && actStat != 0x68) && millis() - timeAfterStopMotors > 500 )
  // {
  //   northOrientation = NorthOrientation();
  // }
  GatewayLink.PendingDataReqSerial[17] = BNOMode;
  if (BNOMode == MODE_IMUPLUS)
  {
    GatewayLink.PendingDataReqSerial[18] = 0x00;
    GatewayLink.PendingDataReqSerial[19] = 0x00;
  }
  if (BNOMode == MODE_COMPASS || getNorthOrientation == 0x01)
  {
    GatewayLink.PendingDataReqSerial[18] = uint8_t(northOrientation / 256);
    GatewayLink.PendingDataReqSerial[19] = uint8_t(northOrientation);
  }
  if (BNOMode == MODE_NDOF)
  {
    GatewayLink.PendingDataReqSerial[18] = uint8_t(absoluteHeading / 256);
    GatewayLink.PendingDataReqSerial[19] = uint8_t(absoluteHeading);
  }
  GatewayLink.PendingDataReqSerial[20] = 0x00;
  GatewayLink.PendingDataReqSerial[21] = lastActionRetcode;
  GatewayLink.PendingDataReqSerial[22] = 0x00;
  GatewayLink.PendingDataReqSerial[23] = toDo;   // for debuging
  GatewayLink.PendingDataReqSerial[24] = 0x00;
  GatewayLink.PendingDataReqSerial[25] = pendingAction; // for debuging
  GatewayLink.PendingDataReqSerial[26] = 0x00;
  GatewayLink.PendingDataReqSerial[27] =  waitFlag; // for debuging
  GatewayLink.PendingDataReqSerial[28] =  lastReceivedNumber; // for debuging
  GatewayLink.PendingDataLenSerial = 0x1d; // 6 longueur mini max 30 pour la gateway
}

void SendPowerValue()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x70; //
  GatewayLink.PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  GatewayLink.PendingDataReqSerial[4] = uint8_t(power3Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[5] = uint8_t(power4Mesurt / 10);
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  GatewayLink.PendingDataReqSerial[7] = uint8_t(power5Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[8] = uint8_t(power6Mesurt / 10);
  GatewayLink.PendingDataLenSerial = 0x09; // 6 longueur mini max 25  pour la gateway
}
void SendEncoderMotorValue()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x71; //
  GatewayLink.PendingDataReqSerial[1] = 0x07;
  GatewayLink.PendingDataReqSerial[2] = uint8_t(leftIncoderHighValue / 256);
  GatewayLink.PendingDataReqSerial[3] = uint8_t(leftIncoderHighValue );
  GatewayLink.PendingDataReqSerial[4] = 0x00;
  GatewayLink.PendingDataReqSerial[5] = uint8_t(leftIncoderLowValue / 256);
  GatewayLink.PendingDataReqSerial[6] = uint8_t(leftIncoderLowValue );
  GatewayLink.PendingDataReqSerial[7] = 0x00;
  GatewayLink.PendingDataReqSerial[8] = uint8_t(rightIncoderHighValue / 256);
  GatewayLink.PendingDataReqSerial[9] = uint8_t(rightIncoderHighValue );
  GatewayLink.PendingDataReqSerial[10] = 0x00;
  GatewayLink.PendingDataReqSerial[11] = uint8_t(rightIncoderLowValue / 256);
  GatewayLink.PendingDataReqSerial[12] = uint8_t(rightIncoderLowValue );
  GatewayLink.PendingDataReqSerial[13] = 0x00;
  GatewayLink.PendingDataReqSerial[14] = uint8_t(leftMotorPWM / 256);
  GatewayLink.PendingDataReqSerial[15] = uint8_t(leftMotorPWM );
  GatewayLink.PendingDataReqSerial[16] = 0x00;
  GatewayLink.PendingDataReqSerial[17] = uint8_t(rightMotorPWM / 256);
  GatewayLink.PendingDataReqSerial[18] = uint8_t(rightMotorPWM );
  GatewayLink.PendingDataReqSerial[19] = 0x00;
  GatewayLink.PendingDataReqSerial[20] = uint8_t(leftToRightDynamicAdjustRatio * 100 / 256);
  GatewayLink.PendingDataReqSerial[21] = uint8_t(leftToRightDynamicAdjustRatio * 100 );
  GatewayLink.PendingDataLenSerial = 0x16; // 6 longueur mini max 25  pour la gateway
}
void SendEncoderValues()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  unsigned int minLeftLevel = Wheels.GetMinLevel(leftWheelId);
  unsigned int maxLeftLevel = Wheels.GetMaxLevel(leftWheelId);
  unsigned int minRightLevel = Wheels.GetMinLevel(rightWheelId);
  unsigned int maxRightLevel = Wheels.GetMaxLevel(rightWheelId);
  GatewayLink.PendingDataReqSerial[0] = 0x72; //
  GatewayLink.PendingDataReqSerial[1] = 0x08;   // paramters number
  GatewayLink.PendingDataReqSerial[2] = uint8_t(leftIncoderHighValue / 256);
  GatewayLink.PendingDataReqSerial[3] = uint8_t(leftIncoderHighValue );
  GatewayLink.PendingDataReqSerial[4] = 0x00;
  GatewayLink.PendingDataReqSerial[5] = uint8_t(maxLeftLevel / 256);
  GatewayLink.PendingDataReqSerial[6] = uint8_t(maxLeftLevel );
  GatewayLink.PendingDataReqSerial[7] = 0x00;
  GatewayLink.PendingDataReqSerial[8] = uint8_t(leftIncoderLowValue / 256);
  GatewayLink.PendingDataReqSerial[9] = uint8_t(leftIncoderLowValue );
  GatewayLink.PendingDataReqSerial[10] = 0x00;
  GatewayLink.PendingDataReqSerial[11] = uint8_t(minLeftLevel / 256);
  GatewayLink.PendingDataReqSerial[12] = uint8_t(minLeftLevel );
  GatewayLink.PendingDataReqSerial[13] = 0x00;
  GatewayLink.PendingDataReqSerial[14] = uint8_t(rightIncoderHighValue / 256);
  GatewayLink.PendingDataReqSerial[15] = uint8_t(rightIncoderHighValue );
  GatewayLink.PendingDataReqSerial[16] = 0x00;
  GatewayLink.PendingDataReqSerial[17] = uint8_t(maxRightLevel / 256);
  GatewayLink.PendingDataReqSerial[18] = uint8_t(maxRightLevel );
  GatewayLink.PendingDataReqSerial[19] = 0x00;
  GatewayLink.PendingDataReqSerial[20] = uint8_t(rightIncoderLowValue / 256);
  GatewayLink.PendingDataReqSerial[21] = uint8_t(rightIncoderLowValue );
  GatewayLink.PendingDataReqSerial[22] = 0x00;
  GatewayLink.PendingDataReqSerial[23] = uint8_t(minRightLevel / 256);
  GatewayLink.PendingDataReqSerial[24] = uint8_t(minRightLevel );
  GatewayLink.PendingDataLenSerial = 0x19; // 6 longueur mini max 25  pour la gateway
}
void SendPWMValues()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x73; //
  GatewayLink.PendingDataReqSerial[1] = 0x03;   // paramters number
  GatewayLink.PendingDataReqSerial[2] = uint8_t(leftMotorPWM / 256);
  GatewayLink.PendingDataReqSerial[3] = uint8_t(leftMotorPWM );
  GatewayLink.PendingDataReqSerial[4] = 0x00;
  GatewayLink.PendingDataReqSerial[5] = uint8_t(rightMotorPWM / 256);
  GatewayLink.PendingDataReqSerial[6] = uint8_t(rightMotorPWM );
  GatewayLink.PendingDataReqSerial[7] = 0x00;
  GatewayLink.PendingDataReqSerial[8] = uint8_t((SlowPWMRatio * 100) / 256);
  GatewayLink.PendingDataReqSerial[9] = uint8_t(SlowPWMRatio * 100);
  GatewayLink.PendingDataLenSerial = 0x0a; // 6 longueur mini max 25  pour la gateway
}
void SendEncodersHolesValues()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x74; //
  GatewayLink.PendingDataReqSerial[1] = 0x02;   // paramters number
  GatewayLink.PendingDataReqSerial[2] = uint8_t(leftWheeelCumulative / 256);
  GatewayLink.PendingDataReqSerial[3] = uint8_t(leftWheeelCumulative);
  GatewayLink.PendingDataReqSerial[4] = 0x3a;
  GatewayLink.PendingDataReqSerial[5] = uint8_t(rightWheeelCumulative / 256);
  GatewayLink.PendingDataReqSerial[6] = uint8_t(rightWheeelCumulative);
  GatewayLink.PendingDataLenSerial = 0x07; // 6 longueur mini max 25  pour la gateway
}

void  SendUDPSubsystemRegister(uint8_t receivedRegister[5], uint8_t receivedValue[5])
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = 0x80; //
  GatewayLink.PendingDataReqSerial[1] = 0x05;   // paramters number
  for (int i = 0; i < 5; i++)
  {
    GatewayLink.PendingDataReqSerial[2 + 3 * i] = receivedRegister[i];
    GatewayLink.PendingDataReqSerial[3 + 3 * i] = receivedValue[i];
    GatewayLink.PendingDataReqSerial[4 + 3 * i] = 0x00;
  }
  GatewayLink.PendingDataLenSerial = 0x12; // 6 longueur mini max 25  pour la gateway
}

void SendScanResultSerial (int distF, int distB)   // send scan echo data to the server
{

  GatewayLink.PendingDataReqSerial[0] = 0x01;
  GatewayLink.PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  GatewayLink.PendingDataReqSerial[2] = 0x46; //"F" front
  GatewayLink.PendingDataReqSerial[3] = uint8_t(distF / 256);
  GatewayLink.PendingDataReqSerial[4] = uint8_t(distF);
  GatewayLink.PendingDataReqSerial[5] = 0x42; //"B" back
  GatewayLink.PendingDataReqSerial[6] = uint8_t(distB / 256);
  GatewayLink.PendingDataReqSerial[7] = uint8_t(distB);
  GatewayLink.PendingDataReqSerial[8] = 0x41; //"A" angle du servo
  GatewayLink.PendingDataReqSerial[9] = uint8_t(AngleDegre / 256); //1er octet contient les facteurs de 256
  GatewayLink.PendingDataReqSerial[10] = uint8_t(AngleDegre); //2eme octets contient le complement - position = Datareq2*256+Datareq3
  GatewayLink.PendingDataReqSerial[11] = 0x00;
  GatewayLink.PendingDataReqSerial[12] = 0x4f;
  GatewayLink.PendingDataReqSerial[13] = uint8_t(northOrientation / 256);
  GatewayLink.PendingDataReqSerial[14] = uint8_t(northOrientation);
  GatewayLink.PendingDataLenSerial = 0x0f;                      // data len
  retryCount = 00;
  pendingAckSerial = 0x01;
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  GatewayLink.SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();                   // init timer used to check for server ack
}
void SendBNOSubsytemStatus()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respBNOSubsytemStatus; //
  GatewayLink.PendingDataReqSerial[1] = BNOMode; //
  GatewayLink.PendingDataReqSerial[2] = BNOCalStat;
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  GatewayLink.PendingDataReqSerial[4] = BNOSysStat;
  GatewayLink.PendingDataReqSerial[5] = BNOSysError;
  GatewayLink.PendingDataLenSerial = 0x06; // 6 longueur mini max 25  pour la gateway
}
void SendBNOLocation ()
{
  trameNumber = trameNumber + 1;
  GatewayLink.PendingDataReqSerial[0] = 0x01;
  GatewayLink.PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  GatewayLink.PendingDataReqSerial[2] = respBNOLocation;
  GatewayLink.PendingDataReqSerial[3] = getBNOLocation;    // used by java to check uptodate (0x00)
  if (BNOLeftPosX >= 0)
  {
    GatewayLink.PendingDataReqSerial[4] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[4] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[5] = uint8_t(abs(round(BNOLeftPosX) / 256));
  GatewayLink.PendingDataReqSerial[6] = uint8_t(abs(round(BNOLeftPosX)));

  if (BNOLeftPosY >= 0)
  {
    GatewayLink.PendingDataReqSerial[7] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[7] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[8] = uint8_t(abs(round(BNOLeftPosY) / 256));
  GatewayLink.PendingDataReqSerial[9] = uint8_t(abs(round(BNOLeftPosY)));

  if (BNORightPosX >= 0)
  {
    GatewayLink.PendingDataReqSerial[10] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[10] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[11] = uint8_t(abs(round(BNORightPosX) / 256));
  GatewayLink.PendingDataReqSerial[12] = uint8_t(abs(round(BNORightPosX)));

  if (BNORightPosY >= 0)
  {
    GatewayLink.PendingDataReqSerial[13] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[13] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[14] = uint8_t(abs(round(BNORightPosY) / 256));
  GatewayLink.PendingDataReqSerial[15] = uint8_t(abs(round(BNORightPosY)));
  if (gyroscopeHeading[gyroscopeHeadingIdx] >= 0)
  {
    GatewayLink.PendingDataReqSerial[16] = 0x2b;
  }
  else
  {
    GatewayLink.PendingDataReqSerial[16] = 0x2d;
  }
  GatewayLink.PendingDataReqSerial[17] = uint8_t(abs(BNOLocationHeading) / 256);
  GatewayLink.PendingDataReqSerial[18] = uint8_t(abs(BNOLocationHeading));
  GatewayLink.PendingDataLenSerial = 0x13;                      // data len
  retryCount = 00;
  pendingAckSerial = 0x01;
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  // DataToSendSerial();
  GatewayLink.SendSecuSerial();                               // secured sending to wait for server ack
  timeSendSecSerial = millis();                   // init timer used to check for server ack
}
void SendNarrowPathMesurments ()
{
  //trameNumber = trameNumber + 1;
  // GatewayLink.PendingDataReqSerial[0] = 0x01;
  //  GatewayLink.PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respNarrowPathMesurments;
  unsigned int passHoles = round((passTrackLeftHoles[0] * leftHoleDistance  +  passTrackRightHoles[0] * rightHoleDistance ) / 2);
  GatewayLink.PendingDataReqSerial[1] = uint8_t(passHoles / 256);
  GatewayLink.PendingDataReqSerial[2] = uint8_t(passHoles);
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  passHoles = round((passTrackLeftHoles[1] * leftHoleDistance  +  passTrackRightHoles[1] * rightHoleDistance) / 2);
  GatewayLink.PendingDataReqSerial[4] = uint8_t(passHoles / 256);
  GatewayLink.PendingDataReqSerial[5] = uint8_t(passHoles);
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  passHoles = round((passTrackLeftHoles[2] * leftHoleDistance  +  passTrackRightHoles[2] * rightHoleDistance) / 2);
  GatewayLink.PendingDataReqSerial[7] = uint8_t(passHoles / 256);
  GatewayLink.PendingDataReqSerial[8] = uint8_t(passHoles);
  GatewayLink.PendingDataReqSerial[9] = 0x00;
  passHoles = round((passTrackLeftHoles[3] * leftHoleDistance  +  passTrackRightHoles[3] * rightHoleDistance) / 2);
  GatewayLink.PendingDataReqSerial[10] = uint8_t(passHoles / 256);
  GatewayLink.PendingDataReqSerial[11] = uint8_t(passHoles);
  GatewayLink.PendingDataReqSerial[12] = 0x00;
  passHoles = round((passTrackLeftHoles[4] * leftHoleDistance  +  passTrackRightHoles[4] * rightHoleDistance) / 2);
  GatewayLink.PendingDataReqSerial[13] = uint8_t(passHoles / 256);
  GatewayLink.PendingDataReqSerial[14] = uint8_t(passHoles);
  GatewayLink.PendingDataReqSerial[15] = 0x00;
  GatewayLink.PendingDataReqSerial[16] = lastPassMonitorStepID;
  GatewayLink.PendingDataReqSerial[17] = 0x00;
  GatewayLink.PendingDataReqSerial[18] = tracePassMonitorStepID;
  GatewayLink.PendingDataReqSerial[19] = traceInterruptByStepID;
  GatewayLink.PendingDataLenSerial = 0x14;                      // data len
}
void SendNarrowPathEchos ()
{
  //  trameNumber = trameNumber + 1;
  // GatewayLink.PendingDataReqSerial[0] = 0x01;
  // GatewayLink.PendingDataReqSerial[1] = uint8_t(trameNumber % 256);
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respNarrowPathEchos;
  for (int i = 0; i < min(9, passNumberTrack1); i++)
  {
    GatewayLink.PendingDataReqSerial[3 * i + 1] = uint8_t(passTrack1[2 * i]);
    GatewayLink.PendingDataReqSerial[3 * i + 2] = uint8_t(passTrack1[2 * i + 1]);
    GatewayLink.PendingDataReqSerial[3 * i + 3] = 0x00;
  }
  GatewayLink.PendingDataLenSerial = uint8_t(min(30, 3 * passNumberTrack1 + 1));                 // data len

}
void SendTraceData()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respTrace; //
  GatewayLink.PendingDataReqSerial[1] = uint8_t(power1Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[2] = uint8_t(power2Mesurt / 10);
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  GatewayLink.PendingDataReqSerial[4] = uint8_t(power3Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[5] = uint8_t(power4Mesurt / 10);
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  GatewayLink.PendingDataReqSerial[7] = uint8_t(power5Mesurt / 10); //
  GatewayLink.PendingDataReqSerial[8] = uint8_t(power6Mesurt / 10);
  GatewayLink.PendingDataReqSerial[9] = 0x00;
  int encoderValue = analogRead(leftAnalogEncoderInput);
#if(wheelEncoderDebugOn)
  Serial.print("left encoder:");
  Serial.print(encoderValue);
#endif
  GatewayLink.PendingDataReqSerial[10] = uint8_t(encoderValue / 256); //
  GatewayLink.PendingDataReqSerial[11] = uint8_t(encoderValue);
  GatewayLink.PendingDataReqSerial[12] = 0x00;
  encoderValue = analogRead(rightAnalogEncoderInput);
#if(wheelEncoderDebugOn)
  Serial.print(" right:");
  Serial.println(encoderValue);
#endif
  GatewayLink.PendingDataReqSerial[13] = uint8_t(encoderValue / 256); //
  GatewayLink.PendingDataReqSerial[14] = uint8_t(encoderValue);
  GatewayLink.PendingDataLenSerial = 0x0f; // 6 longueur mini max 25  pour la gateway
}
void SendTraceNO()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respTraceNO; //
  GatewayLink.PendingDataReqSerial[1] = uint8_t(northOrientation / 256); //
  GatewayLink.PendingDataReqSerial[2] = uint8_t(northOrientation);
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  GatewayLink.PendingDataReqSerial[4] = BNOMode; //
  GatewayLink.PendingDataReqSerial[5] = BNOCalStat;
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  GatewayLink.PendingDataReqSerial[7] = BNOSysStat;
  GatewayLink.PendingDataReqSerial[8] = BNOSysError;
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  GatewayLink.PendingDataReqSerial[10] = uint8_t(absoluteHeading / 256);
  GatewayLink.PendingDataReqSerial[11] = uint8_t(absoluteHeading);
#if(debugMagnetoOn)
  Serial.print("NO:");
  Serial.print(northOrientation);
#endif
  GatewayLink.PendingDataLenSerial = 0x0c; // 6 longueur mini max 25  pour la gateway
}
void SendVersion()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respVersion; //
  GatewayLink.PendingDataReqSerial[1] = ver[0]; //
  GatewayLink.PendingDataReqSerial[2] = ver[1];
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  GatewayLink.PendingDataReqSerial[4] = 0x00; //
  GatewayLink.PendingDataReqSerial[5] = 0x00;
  GatewayLink.PendingDataReqSerial[6] = 0x00;

  GatewayLink.PendingDataLenSerial = 0x06; // 6 longueur mini max 25  pour la gateway
}
void SendPID()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;
  GatewayLink.PendingDataReqSerial[0] = respPID; //
  int work = leftPID.GetKp() * 100;
  GatewayLink.PendingDataReqSerial[1] = uint8_t(work  / 256); //
  GatewayLink.PendingDataReqSerial[2] = uint8_t(work);
  GatewayLink.PendingDataReqSerial[3] = 0x00;
  work = leftPID.GetKi() * 100;
  GatewayLink.PendingDataReqSerial[4] = uint8_t(work  / 256); //
  GatewayLink.PendingDataReqSerial[5] = uint8_t(work);
  GatewayLink.PendingDataReqSerial[6] = 0x00;
  work = leftPID.GetKd() * 100;
  GatewayLink.PendingDataReqSerial[7] = uint8_t(work  / 256); //
  GatewayLink.PendingDataReqSerial[8] = uint8_t(work );
  GatewayLink.PendingDataReqSerial[9] = 0x00;
  GatewayLink.PendingDataReqSerial[10] = uint8_t(outLimit[leftMinOut]);
  GatewayLink.PendingDataReqSerial[11] = uint8_t(outLimit[rightMinOut]);
  GatewayLink.PendingDataReqSerial[12] = 0x00;
  GatewayLink.PendingDataReqSerial[13] = uint8_t(outLimit[leftMaxOut]);
  GatewayLink.PendingDataReqSerial[14] = uint8_t(outLimit[rightMaxOut]);
  GatewayLink.PendingDataReqSerial[15] = 0x00;
  GatewayLink.PendingDataReqSerial[16] = uint8_t(outLimit[leftStartOut]);
  GatewayLink.PendingDataReqSerial[17] = uint8_t(outLimit[rightStartOut]);
  GatewayLink.PendingDataReqSerial[18] = 0x00;
  int setPoint = (int) leftSetpoint;
  GatewayLink.PendingDataReqSerial[19] = uint8_t(setPoint / 256);
  GatewayLink.PendingDataReqSerial[20] = uint8_t(setPoint);
  GatewayLink.PendingDataReqSerial[21] = 0x00;
  setPoint = (int) rightSetpoint;
  GatewayLink.PendingDataReqSerial[22] = uint8_t(setPoint / 256);
  GatewayLink.PendingDataReqSerial[23] = uint8_t(setPoint);
  GatewayLink.PendingDataLenSerial = 0x18; // 6 longueur mini max 25  pour la gateway
}


void TraitInput(uint8_t cmdInput) {     // wet got data on serial
  //  Serial.println("serialInput");
  bitWrite(diagConnection, diagConnectionIP, 0);      // connection is active
  timeReceiveSerial = millis();         // reset check receive timer
  sendInfoSwitch = 1;                   // next info to send will be status
  if (cmdInput != 0x61 && cmdInput != 0x65 && lastReceivedNumber == GatewayLink.DataInSerial[1])
  { // check duplicate for frame that are echo or ack frame
    Serial.print("duplicate receive number:");
    Serial.println(GatewayLink.DataInSerial[1]);
    SendStatus();
    return;
  }
  lastReceivedNumber = GatewayLink.DataInSerial[1];
  switch (cmdInput) {                   // first byte of input is the command type
    case 0x2b: // commande + means scan 360
      if (appStat != 0xff)
      {
        appStat = appStat & 0xf1;
        Serial.println("Scan");
        pulseNumber = 0;
        InitScan(nbPulse, 0);
        actStat = 0x66;
        SetBNOMode(MODE_COMPASS);
        bitWrite(toDo, toDoScan, 1);       // position bit toDo scan
        iddleTimer = millis();
        SendStatus();
      }
      //      SendRFNoSecured();
      break;
    case 0x3a: // commande : power on/off encoder
      Serial.print("commande : power on/off encoder:");
      Serial.println(GatewayLink.DataInSerial[3]);
      digitalWrite(encoderPower, GatewayLink.DataInSerial[3]);
      break;
    case 0x3c: // commande : set encoder threshold
      Serial.print("commande : set threshold encoder ");
      //     Serial.println(GatewayLink.GatewayLink.DataInSerial[3]);
      if (GatewayLink.DataInSerial[3] == 0x4c)
      {
        leftIncoderLowValue = GatewayLink.DataInSerial[5] * 256 + GatewayLink.DataInSerial[6];
        leftIncoderHighValue = GatewayLink.DataInSerial[8] * 256 + GatewayLink.DataInSerial[9];
        Serial.println("left");
      }
      if (GatewayLink.DataInSerial[3] == 0x52)
      {
        rightIncoderLowValue = GatewayLink.DataInSerial[5] * 256 + GatewayLink.DataInSerial[6];
        rightIncoderHighValue = GatewayLink.DataInSerial[8] * 256 + GatewayLink.DataInSerial[9];
        Serial.println("right");
      }
      break;
    case 0x3d: // commande : set motors ratio
      {
        Serial.print("commande : set motors ratio:");
        float inpValue = 0.;
        inpValue = GatewayLink.DataInSerial[3] * 256 + GatewayLink.DataInSerial[4];
        leftToRightDynamicAdjustRatio = inpValue / 100;
        Serial.println(leftToRightDynamicAdjustRatio);
        break;
      }
    case 0x3e: // commande : query encoders values
      {
        Serial.println("commande : query encoders values:");
        SendEncoderValues();
        break;
      }
    case 0x3f: // commande : set motors PMW values
      {
        Serial.print("commande : set motors PMW values :");
        Serial.println(GatewayLink.DataInSerial[3], HEX);
        if (GatewayLink.DataInSerial[3] == 0x4c)
        {
          leftMotorPWM =  GatewayLink.DataInSerial[6];
        }
        if (GatewayLink.DataInSerial[3] == 0x52)
        {
          rightMotorPWM =  GatewayLink.DataInSerial[6];
        }
        if (GatewayLink.DataInSerial[3] == 0x72)
        {
          SlowPWMRatio = (float(GatewayLink.DataInSerial[5]) * 256 + GatewayLink.DataInSerial[6]) / 100;
        }
        break;
      }
    case 0x40: // commande : query encoders values
      {
        Serial.println("commande : query PWM:");
        SendPWMValues();
        break;
      }
    case 0x41: // commande : set gyro selected range
      {
        Serial.print("commande : set gyro selected range :");
        Serial.println(GatewayLink.DataInSerial[3], HEX);
        SetGyroSelectedRange(GatewayLink.DataInSerial[3]);
        break;
      }
    case 0x42: // commande : set gyro ODR
      {
        Serial.print("commande : set gyro ODR :");
        Serial.println(GatewayLink.DataInSerial[3], HEX);
        SetGyroODR(GatewayLink.DataInSerial[3]);
        break;
      }
    case 0x43: // commande : read subsytem register
      {
        Serial.print("commande : read subsytem register:");
        Serial.print(GatewayLink.DataInSerial[3], HEX);
        Serial.print("-");
        uint8_t registers[5];
        for (int i = 0; i < 5; i++)
        {
          registers[i] = GatewayLink.DataInSerial[i + 4];
          Serial.print(registers[i], HEX);
          Serial.print(".");
        }
        Serial.println();
        GetSubsystemRegister(GatewayLink.DataInSerial[3], registers);
        break;
      }
    case 0x44: // commande : set subsystem registers
      {
        uint8_t registers[3];
        uint8_t registersValues[3];
        Serial.print("commande : set GyroBiasMicrosec:");
        Serial.print(GatewayLink.DataInSerial[3], HEX);
        Serial.print(" ");
        for (int i = 0; i < GatewayLink.DataInSerial[3]; i++)
        {
          Serial.print(GatewayLink.DataInSerial[i + 4]);
          Serial.print("=");
          Serial.print(GatewayLink.DataInSerial[i + 5]);
          registers[i] = GatewayLink.DataInSerial[i + 4];
          registersValues[i] = GatewayLink.DataInSerial[i + 5];
        }
        Serial.println();
        SubsystemSetRegisters(max(GatewayLink.DataInSerial[3], 3), registers, registersValues);
        break;
      }

    case 0x45: // E north align
      {
        if (bitRead(toDo, toDoAlign) == 0)   // no pending rotation
        {
          northAligned = false;
          ResetGyroscopeHeadings();
          GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
          northAlignShift = GatewayLink.DataInSerial[3] * 256 + GatewayLink.DataInSerial[4];
          bitWrite(toDo, toDoAlign, 1);       // position bit toDo
          iddleTimer = millis();
#if defined(debugConnection)
          Serial.print("north align:");
          Serial.println(northAlignShift);
#endif
          //       targetAfterNORotation= ((int) alpha + reqN) % 360;
          targetAfterNORotation = 0;
          //       northAlign(reqN);
          break;
        }
      }
    case 0x49: // commande I init robot postion
      {
        posX = GatewayLink.DataInSerial[4] * 256 + GatewayLink.DataInSerial[5]; // received X cm position on 3 bytes including 1 byte for sign
        if (GatewayLink.DataInSerial[3] == 0x2d) {                  // check sign + or -
          posX = -posX;
        }
        posY = GatewayLink.DataInSerial[7] * 256 + GatewayLink.DataInSerial[8]; //received Y cm position on 3 bytes including 1 byte for sign
        if (GatewayLink.DataInSerial[6] == 0x2d) {                 // check sign + or -
          posY = -posY;
        }
        alpha = GatewayLink.DataInSerial[10] * 256 + GatewayLink.DataInSerial[11]; //received orientation (degre) on 3 bytes including 1 byte for sign

        if (GatewayLink.DataInSerial[9] == 0x2d) {                 // check sign + or -
          alpha = -alpha;
        }
        deltaPosX = 0;
        deltaPosY = 0;
        posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180); // x position of the rotation center
        posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180); // y position of the rotation center
        currentLocProb = GatewayLink.DataInSerial[13];   // localisation probability
        sendInfoSwitch = 1;                  // to force next sent info to be status
        SendStatus();
        stepBNOInitLocation = 0x01;
        //      InitBNOLocation();
#if defined(debugConnection)
        Serial.print("init X Y Alpha:");
        Serial.print(posX);
        Serial.print(" ");
        Serial.print(posY);
        Serial.print(" ");
        Serial.println(alpha);
#endif
        break;
      }
    case 0x4f: // commande O obstacle detection
      Serial.print("commande O obstacle detection:");
      obstacleDetectionOn = (GatewayLink.DataInSerial[3]); // 1 on 0 off
      Serial.println(obstacleDetectionOn);
      break;
    case 0x53: // commande S align servo
      Serial.println("align servo");
      EchoServoAlign(GatewayLink.DataInSerial[3], true); // align according to data received - orientation in degres
      iddleTimer = millis();
      break;
    case 0x61: // we received a ack from the server
      lastAckTrameNumber = GatewayLink.DataInSerial[3];
#if defined(debugConnection)
      Serial.print("ack: ");
      for (int i = 0; i < 5; i++)
      {
        Serial.print(GatewayLink.DataInSerial[i], HEX);
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
    case 0x65: // commande e server request for robot status
      if (actStat != 0x66)
      {
        SendStatus();                                     // send robot status to server
      }
      break;

    case 0x67: // commande goto X Y position
      {
        ResetGyroscopeHeadings();
        GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
        int reqX = GatewayLink.DataInSerial[4] * 256 + GatewayLink.DataInSerial[5];
        moveForward = true;           // goto forward
        if (GatewayLink.DataInSerial[3] == 0x2d) {
          reqX = -reqX;
        }
        int reqY = GatewayLink.DataInSerial[7] * 256 + GatewayLink.DataInSerial[8];
        if (GatewayLink.DataInSerial[6] == 0x2d) {
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
        iddleTimer = millis();
        SendStatus();
        ResumeMove();
        break;
      }
    case 0x68: // commande h horn
      {
        unsigned int duration = GatewayLink.DataInSerial[3];
        Horn(true, duration * 1000);
        break;
      }
    case 0x69: // commande i adujst shiftPulse
      {
        Serial.println("set shift pulse:");
        shiftPulse = GatewayLink.DataInSerial[3];
        break;
      }
    case 0x6d: // commande m means move
      bitWrite(diagMotor, diagMotorPbSynchro, 0);       // position bit diagMotor
      bitWrite(diagMotor, diagMotorPbLeft, 0);       // position bit diagMotor
      bitWrite(diagMotor, diagMotorPbRight, 0);       // position bit diagMotor
      gyroTargetRotation = 0;                        // clear previous rotation
      if (appStat != 0xff && bitRead(toDo, toDoMove) == 0 && bitRead(toDo, toDoRotation) == 0)
      {

        toDo = 0x00;
        reqAng = GatewayLink.DataInSerial[4] * 256 + GatewayLink.DataInSerial[5];

        if (GatewayLink.DataInSerial[3] == 0x2d) {
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
        reqMove = GatewayLink.DataInSerial[7] * 256 + GatewayLink.DataInSerial[8];
        if ((abs(reqMove) > 0 && abs(reqMove) < minDistToBeDone) || (abs(reqAng) < 0 && abs(reqAng) < minRotToBeDone))
        {
          SendEndAction(moveEnded, moveUnderLimitation);
          break;
        }
        getBNOLocation = 0xff;
        iddleTimer = millis();
        ResetGyroscopeHeadings();

        appStat = appStat & 0x1f;
        actStat = 0x68; // moving
        bitWrite(toDo, toDoMove, 1);       // position bit toDo move
        bitWrite(toDoDetail, toDoMoveAcrossPass, 0);       // position bit toDodetail
        if (GatewayLink.DataInSerial[6] == 0x2d) {
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
        GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
#if defined(debugGyroscopeOn)
        Serial.print("start gyro monitor:");
        Serial.println(!bitRead(toDo, toDoBackward));
#endif
        SendStatus();
#if defined(debugConnection)
        Serial.print(" ");
        Serial.println(reqMove);
        Serial.print("todo 0x");
        Serial.print(toDo, HEX);
        Serial.print(" waitflag 0x");
        Serial.println(waitFlag, HEX);
#endif
      }
      else
      {
        SendEndAction(moveEnded, requestRejected);
        break;
      }
      break;
    case moveAcrossPass: //
      {
        if (appStat != 0xff && bitRead(toDo, toDoMove) == 0 && bitRead(toDo, toDoRotation) == 0)
        {
          bitWrite(diagMotor, diagMotorPbSynchro, 0);       // position bit diagMotor
          bitWrite(diagMotor, diagMotorPbLeft, 0);       // position bit diagMotor
          bitWrite(diagMotor, diagMotorPbRight, 0);       // position bit diagMotor
          gyroTargetRotation = 0;                        // clear previous rotation
          passDistance = GatewayLink.DataInSerial[4];
          passWidth = GatewayLink.DataInSerial[6];
          passLength = GatewayLink.DataInSerial[8];
          reqMove = (GatewayLink.DataInSerial[10] & 0b01111111) * 256 + GatewayLink.DataInSerial[11];
          passMonitorStepID=0x00;
          passRetCode=0x00;
          if (GatewayLink.DataInSerial[3] == 0x2d) {
            passDistance = -passDistance;
          }
          if (GatewayLink.DataInSerial[9] == 0x2d) {
            reqMove = -reqMove;
          }
          if (reqMove / passDistance < 0)      // must have  the same sign
          {
            SendEndAction(moveAcrossPassEnded, requestRejected);
            break;
          }
          if (max(abs(passDistance), abs(reqMove)) < minDistToBeDone )
          {
            SendEndAction(moveAcrossPassEnded, moveUnderLimitation);
            break;
          }
          echoToGet = (GatewayLink.DataInSerial[13] & 0b01111111) * 256 + GatewayLink.DataInSerial[14];
          iddleTimer = millis();

#if defined(debugAcrossPathOn)
          Serial.print("move across pass dist:");
          Serial.print( passDistance);
          Serial.print(" width:");
          Serial.print( passWidth);
          Serial.print(" length:");
          Serial.print( passLength);
          Serial.print(" reqMove:");
          Serial.print( reqMove);
          Serial.print(" echoToGet:");
          Serial.println( echoToGet);
#endif
          bitWrite(toDoDetail, toDoMoveAcrossPass, true);       // position bit toDo
          bitWrite(toDo, toDoMove, true);
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

        }
        else
        {
          SendEndAction(moveAcrossPassEnded, requestRejected);
          break;
        }
        break;
      }
    case rotateTypeGyro: // r rotate VS gyroscope
      {
        int reqN = ((GatewayLink.DataInSerial[3] & 0b01111111) * 256 + GatewayLink.DataInSerial[4]) % 360;
        if (bitRead(toDoDetail, toDoGyroRotation) == 0)   // no pending rotation
        {
          if (abs(reqN) < minRotGyroAbility )
          {
            SendEndAction(moveEnded, moveUnderLimitation);
            break;
          }
          iddleTimer = millis();
          rotationType = cmdInput;
          posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180);  // save rotation center x position
          posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180);  // save rotation center y position
          ResetGyroscopeHeadings();
          GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
          delay(100);


          if (bitRead(GatewayLink.DataInSerial[3], 7) == 1)    // means negative rotation
          {
            reqN = -reqN;
            gyroInitRotationSens = -1;
          }
          else
          {
            gyroInitRotationSens = 1;
          }
#if defined(debugGyroscopeOn)
          Serial.print("request gyro rotate:");
          Serial.println( reqN);
#endif
          timeGyroRotation = millis();
          gyroTargetRotation = reqN;
          bitWrite(toDoDetail, toDoGyroRotation, 1);       // position bit toDo
          gyroRotationRetry = 0;
          iddleTimer = millis();
          rotationType = cmdInput;
          posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180);  // save rotation center x position
          posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180);  // save rotation center y position
          ResetGyroscopeHeadings();
          GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
          delay(100);

        }
        else
        {
          SendEndAction(moveEnded, requestRejected);
          break;
        }
        break;
      }
    case pingFrontBack: // commande p ping front back
      iddleTimer = millis();
      Serial.println("ping FB");
      bitWrite(toDo, toDoPingFB, 1);
      PingFrontBack(); //
      break;
    case 0x73: // commande s means stop
      Serial.println("cmd stop");
      ResetGyroscopeHeadings();
      //    GyroStopInitMonitor();
      StopAll();
      break;
    case 0x72: // commande r menus reset
      Serial.print("cmd reset:");
      Serial.println(GatewayLink.DataInSerial[3], HEX);
      ResetGyroscopeHeadings();
      GyroStopInitMonitor();
      resetStatus(GatewayLink.DataInSerial[3]); // reset according to data receveid
      SendStatus();
      break;
    case 0x77: // commande w means calibrate wheels
      iddleTimer = millis();
      Serial.println("calibrate");
      ResetGyroscopeHeadings();
      GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
      CalibrateWheels();
      break;
    case 0x78: // commande x menus start
      iddleTimer = millis();
      Serial.println("cmd start");
      appStat = 0x00;
      resetStatus(0xff);     // reset all
      ResetGyroscopeHeadings();
      //     GyroStartInitMonitor();
      SendStatus();
      break;
    case 0x79: // commande x menus start
      iddleTimer = millis();
      SetBNOMode(GatewayLink.DataInSerial[3]);
      Serial.print("set BNOMode");
      Serial.println(GatewayLink.DataInSerial[3]);
      break;
    case 0x7a: // get subsytem location
      getBNOLocation = 0x07;
      Serial.println("getBNOLocation");
      break;
    case requestUpdateNO: // get north orientation
      getNorthOrientation = 0x02;
      compasUpToDate = 0x00;
      Serial.println("getNorthOrientation");
      break;
    case requestBNOData: // send
      SendBNOLocation ();
      Serial.println("SendBNOLocation");
      break;
    default:

      Serial.print("commande recue: ");
      Serial.println(cmdInput, HEX);
  }


}

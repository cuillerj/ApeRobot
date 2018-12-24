void TraitInput(uint8_t cmdInput) {     // wet got data on serial
  //  Serial.println("serialInput");
  bitWrite(diagConnection, diagConnectionIP, 0);      // connection is active
  timeReceiveSerial = millis();         // reset check receive timer
  sendInfoSwitch = 1;                   // next info to send will be status
  if (cmdInput != 0x61 && cmdInput != 0x65 && lastReceivedNumber == GatewayLink.DataInSerial[1])
  { // check duplicate for frame that are echo or ack frame
    Serial.print("duplicate received frame number:");
    Serial.println(GatewayLink.DataInSerial[1]);
    SendStatus();
    return;
  }
  lastReceivedNumber = GatewayLink.DataInSerial[1];
  switch (cmdInput) {                   // first byte of input is the command type
    case 0x2b: // commande + means scan 360
      if (appStat != 0xff && !bitRead(toDo, toDoScan))
      {
        appStat = appStat & 0xf1;
        Serial.println("Scan");
        pulseNumber = 0;
        int startOriention = 0;
        /*
          if (defaultServoOrientation == -1)
          {
          startOriention = nbPulse-1;
          pulseNumber=nbPulse-1;
          }
        */
        InitScan(nbPulse, startOriention);
        actStat = scan360;
        //       SetBNOMode(MODE_COMPASS);
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
    case 0x3f: // commande : set motors PWM values
      {
        Serial.print("commande : set motors PWM values :");
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
    case 0x40: // commande : query PWM values
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

    case northAlignRequest: // E north align
      {
        //        if (bitRead(toDo, toDoAlign) == 0)   // no pending rotation
        //        {
        actStat = northAlignRequest;
        bitWrite(toDoDetail, toDoAlignRotate, 1);
        bitWrite(toDoDetail, toDoAlignUpdateNO, 0);
        //         northAlignTarget = GatewayLink.DataInSerial[3] * 256 + GatewayLink.DataInSerial[4];
        initNorthAlign((unsigned int)(GatewayLink.DataInSerial[3] * 256 + GatewayLink.DataInSerial[4]));
#if defined(debugConnection)
        Serial.print("north align:");
        Serial.println(northAlignTarget);
#endif
        /*
          northAligned = false;
          ResetGyroscopeHeadings();
          GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
          bitWrite(toDo, toDoAlign, 1);       // position bit toDo
          iddleTimer = millis();
          #if defined(debugConnection)
          Serial.print("north align:");
          Serial.println(northAlignShift);
          #endif
          //       targetAfterNORotation= ((int) alpha + reqN) % 360;
          targetAfterNORotation = 0;
        */
        //       northAlign(reqN);
        break;
        //       }
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
    case 0x50: //
      Serial.print("set pulseLenght:");
      pulseLenght = (GatewayLink.DataInSerial[4]); //
      Serial.println(pulseLenght);
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
      if (actStat != scan360)
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
        actStat = moving; // moving
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
      if (appStat != 0xff && bitRead(toDo, toDoMove) == 0 && bitRead(toDo, toDoRotation) == 0 && !encodersToStop)
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
        actStat = moving; // moving
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
        actStat = moveAcrossPass;
        if (appStat != 0xff && bitRead(toDo, toDoMove) == 0 && bitRead(toDo, toDoRotation) == 0)
        {
          bitWrite(diagMotor, diagMotorPbSynchro, 0);       // position bit diagMotor
          bitWrite(diagMotor, diagMotorPbLeft, 0);       // position bit diagMotor
          bitWrite(diagMotor, diagMotorPbRight, 0);       // position bit diagMotor
          tracePassMonitorStepID = 0x00;
          traceInterruptByStepID = 0x00;
          gyroTargetRotation = 0;                        // clear previous rotation
          passDistance = GatewayLink.DataInSerial[4];
          passWidth = GatewayLink.DataInSerial[6];
          passLen = GatewayLink.DataInSerial[8];
          reqMove = (GatewayLink.DataInSerial[10] & 0b01111111) * 256 + GatewayLink.DataInSerial[11];
          passMonitorStepID = 0x00;
          passRetCode = 0x00;
          for (int i = 0; i < nbTrackedHoles; i++) {
            passTrackLeftHoles[i] = 0x00;
            passTrackRightHoles[i] = 0x00;
          }
          for (int i = 0; i < 2 * passNumberTrack1; i++) {
            passTrack1[i] = 0x00;
          }
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
          passNO = (GatewayLink.DataInSerial[16] & 0b01111111) * 256 + GatewayLink.DataInSerial[17];
          passStartEntryDistance = GatewayLink.DataInSerial[19];
          iddleTimer = millis();

#if defined(debugAcrossPathOn)
          Serial.print("move across pass dist:");
          Serial.print( passDistance);
          Serial.print(" width:");
          Serial.print( passWidth);
          Serial.print(" length:");
          Serial.print( passLen);
          Serial.print(" reqMove:");
          Serial.print( reqMove);
          Serial.print(" echoToGet:");
          Serial.print( echoToGet);
          Serial.print(" passNO:");
          Serial.print( passNO);
          Serial.print(" startToEntryDistance:");
          Serial.println( passStartEntryDistance);
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
        actStat = gyroRotating;
        int reqN = ((GatewayLink.DataInSerial[3] & 0b01111111) * 256 + GatewayLink.DataInSerial[4]) % 360;
        if (reqN == 0 )
        {
          SendEndAction(moveEnded, 0x00);
          break;
        }
        if (bitRead(toDoDetail, toDoGyroRotation) == 0)   // no pending rotation
        {
          if (abs(reqN) < minRotGyroAbility )
          {
            SendEndAction(moveEnded, moveUnderLimitation);
            break;
          }
          iddleTimer = millis();
          rotationType = cmdInput;
          //       posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180);  // save rotation center x position
          //       posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180);  // save rotation center y position
          //       ResetGyroscopeHeadings();
          //       GyroStartInitMonitor(!bitRead(toDo, toDoBackward));
          //       delay(100);


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
          bitWrite(toDo, toDoRotation, 1);       // position bit toDo
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
        SendStatus();
        break;
      }
    case requestPingFrontBack: // commande p ping front back
      if (!bitRead(toDo, toDoScan))
      {
        iddleTimer = millis();
        Serial.println("ping FB");
        bitWrite(toDo, toDoPingFB, 1);
        PingFrontBack(); //
      }
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
      actStat = requestUpdateNO;
      getNorthOrientation = 0x00;
      compasUpToDate = 0x00;
      saveNorthOrientation = 999;
      bitWrite(toDo, toDoAlign, 1);
      bitWrite(toDoDetail, toDoAlignRotate, 0);
      bitWrite(toDoDetail, toDoAlignUpdateNO, 1);
      Serial.println("getNorthOrientation");
      sendInfoSwitch = 1;        // for sending status priority
      timeSendInfo = millis() + delayBetweenInfo; // for delaying the next send
      break;
    case requestBNOData: // send
      SendBNOLocation ();
      Serial.println("SendBNOLocation");
      break;
    case requestNarrowPathMesurments: //
      SendNarrowPathMesurments ();
      Serial.println("requestNarrowPathMesurments");
      break;
    case requestNarrowPathEchos: //
      SendNarrowPathEchos ();
      Serial.println("requestNarrowPathEchos");
      break;
    case requestTrace: //
      if (GatewayLink.DataInSerial[3] == 0x00)
      {
        bitWrite(appTrace, traceBitPower, 0);
        bitWrite(appTrace, traceBitEncoder, 0);
        bitWrite(appTrace, traceBitRequest, 0);
        Serial.println("stopTrace");
      }
      else {
        bitWrite(appTrace, traceBitPower, 1);
        bitWrite(appTrace, traceBitEncoder, 1);
        bitWrite(appTrace, traceBitRequest, 1);
        Serial.println("startTrace");
      }
      break;
    case requestTraceNO: //
      if (GatewayLink.DataInSerial[3] == 0x00)
      {
        bitWrite(appTrace, traceBitNO, 0);
        bitWrite(appTrace, traceBitRequest, 0);
        Serial.println("stopTraceNO");
      }
      else {
        bitWrite(appTrace, traceBitNO, 1);
        bitWrite(appTrace, traceBitRequest, 1);
        Serial.println("startTraceNO");
      }
      break;
    case requestSleep: //
      if (GatewayLink.DataInSerial[3] == 0x01)
      {
        Serial.println("requestSleep");
        delay(100);
        sleepRequest = true;
      }
      break;
    case requestVersion:
      {
        SendVersion();
        break;
      }
    case requestPID:
      {
        SendPID();
        break;
      }
    case setPID:
      {
        Serial.print("SetPID");
#define setKx 0
#define setInput 1
#define setSetpoint 2
        switch (GatewayLink.DataInSerial[3])
        {
          case setKx:
            Serial.print(" Kx:");
            Serial.println(GatewayLink.DataInSerial[4]);
            if (GatewayLink.DataInSerial[4] >= 0 && GatewayLink.DataInSerial[4] < sizeOfKx)
            {
              Kx[GatewayLink.DataInSerial[4]] = (GatewayLink.DataInSerial[5] & 0b01111111) * 256 + GatewayLink.DataInSerial[6];
              Kx[GatewayLink.DataInSerial[4]] = Kx[GatewayLink.DataInSerial[4]] / 100;
              SetPIDKx();
              break;
            }
          case setInput:
            Serial.print(" input:");
            Serial.println(GatewayLink.DataInSerial[4]);
            if (GatewayLink.DataInSerial[4] >= 0 && GatewayLink.DataInSerial[4] < sizeOfOutlim)
            {
              outLimit[GatewayLink.DataInSerial[4]] = GatewayLink.DataInSerial[5];
              leftPID.SetOutputLimits(outLimit[leftMinOut], outLimit[leftMaxOut]);
              rightPID.SetOutputLimits(outLimit[rightMinOut], outLimit[rightMaxOut]);
              break;
            }
          case setSetpoint:
            Serial.print(" setLeftSetpoint:");
            Serial.println(GatewayLink.DataInSerial[4]);
            if (GatewayLink.DataInSerial[4] == 0)
            {
              leftSetpoint = (GatewayLink.DataInSerial[5] & 0b01111111) * 256 + GatewayLink.DataInSerial[6];
            }
            if (GatewayLink.DataInSerial[4] == 1)
            {
              rightSetpoint = (GatewayLink.DataInSerial[5] & 0b01111111) * 256 + GatewayLink.DataInSerial[6];
            }
            break;

        }
        SendPID();
        break;
      }
    default:
      Serial.print("commande recue: ");
      Serial.println(cmdInput, HEX);
  }


}

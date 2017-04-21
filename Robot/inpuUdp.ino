
void TraitInput(uint8_t cmdInput) {     // wet got data on serial
  //  Serial.println("serialInput");
  bitWrite(diagConnection, diagConnectionIP, 0);      // connection is active
  timeReceiveSerial = millis();         // reset check receive timer
  sendInfoSwitch = 1;                   // next info to send will be status
  lastReceivedNumber = DataInSerial[1];
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
      Serial.println(DataInSerial[3]);
      digitalWrite(encoderPower, DataInSerial[3]);
      break;
    case 0x3c: // commande : set encoder threshold
      Serial.print("commande : set threshold encoder ");
      //     Serial.println(DataInSerial[3]);
      if (DataInSerial[3] == 0x4c)
      {
        leftIncoderLowValue = DataInSerial[5] * 256 + DataInSerial[6];
        leftIncoderHighValue = DataInSerial[8] * 256 + DataInSerial[9];
        Serial.println("left");
      }
      if (DataInSerial[3] == 0x52)
      {
        rightIncoderLowValue = DataInSerial[5] * 256 + DataInSerial[6];
        rightIncoderHighValue = DataInSerial[8] * 256 + DataInSerial[9];
        Serial.println("right");
      }
      break;
    case 0x3d: // commande : set motors ratio
      {
        Serial.print("commande : set motors ratio:");
        float inpValue = 0.;
        inpValue = DataInSerial[3] * 256 + DataInSerial[4];
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
        Serial.println(DataInSerial[3], HEX);
        if (DataInSerial[3] == 0x4c)
        {
          leftMotorPWM =  DataInSerial[6];
        }
        if (DataInSerial[3] == 0x52)
        {
          rightMotorPWM =  DataInSerial[6];
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
        Serial.println(DataInSerial[3], HEX);
        SetGyroSelectedRange(DataInSerial[3]);
        break;
      }
    case 0x42: // commande : set gyro ODR
      {
        Serial.print("commande : set gyro ODR :");
        Serial.println(DataInSerial[3], HEX);
        SetGyroODR(DataInSerial[3]);
        break;
      }
    case 0x43: // commande : read subsytem register
      {
        Serial.print("commande : read subsytem register:");
        Serial.print(DataInSerial[3], HEX);
        Serial.print("-");
        uint8_t registers[5];
        for (int i = 0; i < 5; i++)
        {
          registers[i] = DataInSerial[i + 4];
          Serial.print(registers[i], HEX);
          Serial.print(".");
        }
        Serial.println();
        GetSubsystemRegister(DataInSerial[3], registers);
        break;
      }
    case 0x44: // commande : set subsystem registers
      {
        uint8_t registers[3];
        uint8_t registersValues[3];
        Serial.print("commande : set GyroBiasMicrosec:");
        Serial.print(DataInSerial[3], HEX);
        Serial.print(" ");
        for (int i = 0; i < DataInSerial[3]; i++)
        {
          Serial.print(DataInSerial[i + 4]);
          Serial.print("=");
          Serial.print(DataInSerial[i + 5]);
          registers[i] = DataInSerial[i + 4];
          registersValues[i] = DataInSerial[i + 5];
        }
        Serial.println();
        SubsystemSetRegisters(max(DataInSerial[3], 3), registers, registersValues);
        break;
      }

    case 0x45: // E north align
      {
        northAligned = false;
        ResetGyroscopeHeadings();
        GyroStartInitMonitor();
        northAlignShift = DataInSerial[3] * 256 + DataInSerial[4];
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
    case 0x49: // commande I init robot postion
      {
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
        posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180); // x position of the rotation center
        posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180); // y position of the rotation center
        currentLocProb = DataInSerial[13];   // localisation probability
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
      obstacleDetectionOn = (DataInSerial[3]); // 1 on 0 off
      Serial.println(obstacleDetectionOn);
      break;
    case 0x53: // commande S align servo
      Serial.println("align servo");
      EchoServoAlign(DataInSerial[3], true); // align according to data received - orientation in degres
      iddleTimer = millis();
      break;
    case 0x61: // we received a ack from the server
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
    case 0x65: // commande e server request for robot status
      if (actStat != 0x66)
      {
        SendStatus();                                     // send robot status to server
      }
      break;

    case 0x67: // commande goto X Y position
      {
        ResetGyroscopeHeadings();
        GyroStartInitMonitor();
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
        iddleTimer = millis();
        SendStatus();
        ResumeMove();
        break;
      }
    case 0x68: // commande h horn
      {
        Serial.println("horn");
        unsigned int duration = DataInSerial[3];
        Horn(true, duration * 1000);
        break;
      }
    case 0x69: // commande i adujst shiftPulse
      {
        Serial.println("set shift pulse:");
        shiftPulse = DataInSerial[3];
        break;
      }
    case 0x6d: // commande m means move
      if (appStat != 0xff)
      {
        getBNOLocation = 0xff;
        iddleTimer = millis();
        ResetGyroscopeHeadings();
        GyroStartInitMonitor();
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
      break;
    /*
        case 0x6e: // r rotate VS north
          {
            ResetGyroscopeHeadings();
            GyroStartInitMonitor();
            unsigned int reqN = DataInSerial[3] * 256 + DataInSerial[4];

      #if defined(debugConnection)
            Serial.print("north rotate:");
            Serial.println( reqN);
      #endif
            targetAfterNORotation = ((int) alpha + reqN) % 360;
            northAlign((NorthOrientation() - reqN + 360) % 360);
            break;
          }
    */
    case 0x6f: // r rotate VS gyroscope
      {
        iddleTimer = millis();
        rotationType = cmdInput;
        posRotationGyroCenterX = posX - shiftEchoVsRotationCenter * cos(alpha * PI / 180);  // save rotation center x position
        posRotationGyroCenterY = posY - shiftEchoVsRotationCenter * sin(alpha * PI / 180);  // save rotation center y position
        ResetGyroscopeHeadings();
        GyroStartInitMonitor();
        delay(100);

        int reqN = ((DataInSerial[3] & 0b01111111) * 256 + DataInSerial[4]) % 360;
        if (bitRead(DataInSerial[3], 7) == 1)    // means negative rotation
        {
          reqN = -reqN;
          gyroInitRotationSens = -1;
        }
        else
        {
          gyroInitRotationSens = 1;
        }
        Serial.print("request gyro rotate:");
        Serial.println( reqN);
        timeGyroRotation = millis();
        gyroTargetRotation = reqN;
        if (CheckRotationAvaibility(gyroTargetRotation))    // check rotation
        {
          bitWrite(toDoDetail, toDoGyroRotation, 1);       // position bit toDo
          Serial.print("request gyro rotate:");
          Serial.println( gyroTargetRotation);
          gyroRotationRetry = 0;
          //       uint8_t retCode = GyroscopeRotate();
          //        targetAfterGyroRotation = ((int) alpha + reqN) % 360;
          //       northAlign((NorthOrientation() - reqN + 360) % 360);
        }
        break;
      }
    case 0x70: // commande p ping front back
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
      SendStatus();
      break;
    case 0x72: // commande r menus reset
      Serial.print("cmd reset:");
      Serial.println(DataInSerial[3], HEX);
      ResetGyroscopeHeadings();
      GyroStopInitMonitor();
      resetStatus(DataInSerial[3]); // reset according to data receveid
      SendStatus();
      break;
    case 0x77: // commande w means calibrate wheels
      iddleTimer = millis();
      Serial.println("calibrate");
      ResetGyroscopeHeadings();
      GyroStartInitMonitor();
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
      SetBNOMode(DataInSerial[3]);
      Serial.print("set BNOMode");
      Serial.println(DataInSerial[3]);
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
    default:

      Serial.print("commande recue: ");
      Serial.println(cmdInput, HEX);
  }


}

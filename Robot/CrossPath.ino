void MoveAcrossPath()
{
  /*
    intCode = 0x00 first call 0x01 loop call 0x02  move ended call

    straight move inside a specific pass
    passDistance: maximum distance the pass is expected to start
    passWidth: width off the pass
    passStartEntryDistance: length of the pass
    reqMove: the expected distance to move - if 0 it means robot must stop at the end of the pass
    echoToGet: the echo distance over that we are getting in the pass

    step 0 is used to enventually north aligned the robot
    step 1 & 2 move straight untill front echo distance is over echoToGet (0 meaning over) which means we got the path start
    maximum distance is defined by passDistance - in this case stop and report
    step 3 move straignt until aside echo sum distances is equal to passWidth
    maximum distance is defined by passStartEntryDistance  - in this case stop and report
    step 4 & 5 move straignt until aside echo sum distances is not equal to passWidth
    maximum distance is defined by passLen  - in this case stop and report
    step 6 & 7 move straight reqMove

    firstly the robot will move straight pinging continously ahead till reaching a point where echo distance is over echoToGet (0 meaning over)
    in case going farther than passDistance without reaching this point stop and send report
    passMonitorStepID 0xYZ  Y: action (monitor bit7, request bit6)  Z:step number

  */

#define eFront 0          // echo front identifer for echo monitoring
#define eBack 1

  uint8_t echoID = 0x00;
  uint8_t action = 0x00;

  if (passMonitorStepID != lastPassMonitorStepID)
  {
#if defined(debugAcrossPathOn)
    Serial.print("encodersToStop:");
    Serial.print(encodersToStop, HEX);
    Serial.print(" passInterruptBy:");
    Serial.print(passInterruptBy, HEX);
    Serial.print(" passMonitorStepID:");
    Serial.print(passMonitorStepID, HEX);
    Serial.print(" lastPassMonitorStepID:");
    Serial.print(lastPassMonitorStepID, HEX);
    Serial.print(" ");
    Serial.println(millis());
#endif
    lastPassMonitorStepID = passMonitorStepID; // keep track of the previous step

  }
  if (passMonitorStepID != 0x00 && passMonitorStepID != passMonitorIddle && (millis() > timePassMonitorStarted + maxPassMonitorDuration)) // check not exceed limit duration
  {
#if defined(debugAcrossPathOn)
    Serial.print("passMonitor timeout step: 0x");
    Serial.println(passMonitorStepID, HEX);
    Serial.print(" encodersToStop: ");
    Serial.print(encodersToStop, HEX);
    Serial.print(" Whl: ");
    Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
    Serial.print(" Whr: ");
    Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
    for (int i = 0; i < 5; i++)
    {
      Serial.print("Lh: ");
      Serial.print(passTrackLeftHoles[i]);
      Serial.print(" Rh: ");
      Serial.println(passTrackRightHoles[i]);

    }
    for (int i = 0; i < passNumberTrack1; i++)
    {
      Serial.print("echo F: ");
      Serial.print(passTrack1[2 * i]);
      Serial.print(" B: ");
      Serial.println(passTrack1[2 * i + 1]);
    }
#endif
    passMonitorStepID = passMonitorIddle;
    toDo = 0x00;                                         // clear flag todo
    toDoDetail = 0x00;                                         // clear flag todo
    //    uint8_t retCode = 0x99;
    InterruptMoveAcrossPath(timeoutRetCode);
    return;
  }

  switch (passMonitorStepID)
  {
    case 0x00:
      {
#if defined(debugAcrossPathOn)
        Serial.println("Init MoveAcrossPath");
#endif
        timePassMonitorStarted = millis();
        //    initNorthAlign(passNO);
        passMonitorStepID++;
        bitWrite(passMonitorStepID, passMonitorRequestBit, 1);
        obstacleDetectionOn = false;           // disable standard obstacle detection during move
        break;
      }
    case 0x41:                                 // wait for north alignment
      {
#if defined(debugAcrossPathOn)
        Serial.println("check north alignement");
#endif
        if ( bitRead(toDo, toDoAlign) == true) {
#if defined(debugAcrossPathOn)
          Serial.println("north aligned");
#endif
          bitWrite(passMonitorStepID, passMonitorRequestBit, 0);
          bitWrite(passMonitorStepID, passMonitorInterruptBit, 1);
          return;
        }
      }
    case 0x81:                                 // north aligned look for path start
      {
        /*
          move toward pass start point
        */
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif
        passMonitorStepID = passMonitorStepID & 0x0f;
        passTrackLeftHoles[0] = Wheels.GetCurrentHolesCount(leftWheelId);
        passTrackRightHoles[0] = Wheels.GetCurrentHolesCount(rightWheelId);
        timePassMonitorStarted = millis();
        EchoServoAlign(90, false); // align servo with robot
        int echoDist = 0;
        if (bitRead(toDo, toDoBackward))
        {
          echoDist = PingBack();
          echoID = 0b00000010;
        }
        else {
          echoDist = PingFront();
          echoID = 0b00000001;
        }
        if (echoDist >= echoToGet)  //  already further than the pass start
        {
          bitWrite(passMonitorStepID, passSkipStepBit, 1); // goto next step
#if defined(debugAcrossPathOn)
          Serial.print("skip step 2 echoDist: ");
          Serial.println(echoDist);
#endif
        }
        else {
          if (!encodersToStop) {
            MoveForward(passDistance);
            action = 0x00;
            bitWrite(action, actionUpper, 1);
            StartEchoPassMonitor(passMonitorStepID, echoID, echoToGet , action, 0);
          }
        }
        passMonitorStepID++;
        break;
      }

    default:
      {
        break;
      }
    case 0x12:                                 // looking for path start has been skipped
      {
        break;
      }
    case 0x42:                                 // looking for pass entry
      {
        if (!bitRead(currentMove, toDoStraight))       // is move ended without getting path entry ?
        {
#if defined(debugAcrossPathOn)
          Serial.println("path entry not found");
#endif
          StopMonitorInterrupt();
          actStat = moveAcrossPassEnded;                                      // status move completed
          toDo = 0x00;
          passInterruptBy = 0x00;                                   // clear move flag
          passRetCode = moveAcrossPathKoDueToNotFindingStart;       // store move across path return code
          InterruptMoveAcrossPath(moveAcrossPathKoDueToNotFindingStart);
          passMonitorStepID = 0x2e;
          //       timePassMonitorStarted = millis();                        // to wait a little bit to complete location calculation
          //      getBNOLocation = 0x07;                // to resquest BNO computed location
        }
        break;
      }
    case 0x92:                                  // got entry point entry
    case 0x82:                                 // got entry point entry
      {
#if defined(debugAcrossPathOn)
        Serial.println("got entry point");
#endif
        /*
          we most likely have reached pass start point
        */
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif

        if (!bitRead(passMonitorStepID, passSkipStepBit))
        {
          InterruptMoveAcrossPath(0x00);
        }
        else {
          bitWrite(tracePassMonitorStepID, (passMonitorStepID & 0b00000111), 0);
          bitWrite(passMonitorStepID, passSkipStepBit, 0);
        }
        passTrackLeftHoles[1] = Wheels.GetCurrentHolesCount(leftWheelId);
        passTrackRightHoles[1] = Wheels.GetCurrentHolesCount(rightWheelId);
        bitWrite(passMonitorStepID, passMonitorInterruptBit, 0); // clear interrupt flag
        bitWrite(passMonitorStepID, passMonitorRequestBit, 0);
        passMonitorStepID++;
        break;
      }
    case 0x03:
      {
        /*
          move forward checking width to detect pass path entry -
        */
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif
        if (!encodersToStop) {
          timePassMonitorStarted = millis();
          EchoServoAlign(0, false);
          echoID = 0b00000011;
          action = echoID;
          /*
            monitor side echos till getting summ equal to pass width
          */
          bitWrite(action, actionEqual, 1);
          //        passTrackNumber = 0;
          StartEchoPassMonitor((passMonitorStepID & 0x0f), echoID, passWidth , action, echoPrecision + passWidth * echoTolerance);
          MoveForward(passStartEntryDistance);
        }
        break;
      }
    case 0x43:             // checking path width
      {
        if (!bitRead(currentMove, toDoStraight))       // is move ended without getting end of path ?
        {
#if defined(debugAcrossPathOn)
          Serial.println("path entry not found");
#endif
          StopMonitorInterrupt();
          actStat = moveAcrossPassEnded;                                      // status move completed
          toDo = 0x00;
          passInterruptBy = 0x00;                                   // clear move flag
          passRetCode = moveAcrossPathKoDueToNotFindingEntry;
          InterruptMoveAcrossPath(moveAcrossPathKoDueToNotFindingEntry);
          passMonitorStepID = 0x2e;
          //        timePassMonitorStarted = millis();
          //      getBNOLocation = 0x07;                // to resquest BNO computed location
        }
        break;
      }
    case 0x83:             // got path width
      {
        /*
          we are most likely have reach the path entry
          keep track of some mesurments and then stop move
          and monitor side echos till getting sum not equal to pass width
        */
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif
        passInterruptBy = 0x00;
        uint8_t distF = echo.GetPrevDistance(eFront);
        uint8_t distB = echo.GetPrevDistance(eBack);
        StopMonitorInterrupt();
        passTrackNumber = 0;
        passTrack1[passTrackNumber] = distF;
        passTrack1[passTrackNumber + 1] = distB;
        passTrackNumber = passTrackNumber + 2;
#if defined(debugAcrossPathOn)
        Serial.print("keep track1 F: ");
        Serial.print(distF);
        Serial.print(" B: ");
        Serial.print(distB);
        Serial.print(" - ");
        Serial.println(passTrackNumber);
#endif

        passMonitorStepID = passMonitorStepID & 0x0f;
        passMonitorStepID++;

        passTrackLeftHoles[2] = Wheels.GetCurrentHolesCount(leftWheelId);
        passTrackRightHoles[2] = Wheels.GetCurrentHolesCount(rightWheelId);
        InterruptMoveAcrossPath(0x00);
        break;
      }
    case 0x04:   // check witdh for end of path
      {
        delay(1000);
        echoID = 0b00000011;
        action = echoID;
        bitWrite(action, actionUpper, 1);
        StartEchoPassMonitor(passMonitorStepID, echoID, passWidth , action, echoPrecision + passWidth * echoTolerance);
        MoveForward(passLen);
        bitWrite(passMonitorStepID, passMonitorRequestBit, 1);
      }
    case 0x44:   // check witdh for end of path
      {
        if (!bitRead(currentMove, toDoStraight))       // is move ended without getting end of path ?
        {
#if defined(debugAcrossPathOn)
          Serial.println("path exit not found");
#endif
          StopMonitorInterrupt();
          actStat = moveAcrossPassEnded;                                      // status move completed
          toDo = 0x00;
          passInterruptBy = 0x00;                                   // clear move flag
          passRetCode = moveAcrossPathKoDueToNotFindingExit;
          InterruptMoveAcrossPath(moveAcrossPathKoDueToNotFindingExit);
          passMonitorStepID = 0x2e;
        }
        else {
          /*
            we are most likely inside the path
            keep moving and track of some mesurments
          */
          //     bitWrite(tracePassMonitorStepID, 4, 1);
#if defined(debugAcrossPathOn)
          Serial.print("MoveAcrossPath: 0x");
          Serial.print(passMonitorStepID, HEX);
          Serial.print(" encodersToStop: ");
          Serial.print(encodersToStop, HEX);
          Serial.print(" Whl: ");
          Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
          Serial.print(" Whr: ");
          Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif
          if (passTrackNumber < 2 * passNumberTrack1 && (millis() - lastEchoTime > echoMonitorCycleDuration * 2000))
          {
            uint8_t distF = echo.GetPrevDistance(eFront);
            uint8_t distB = echo.GetPrevDistance(eBack);
#if defined(debugAcrossPathOn)
            Serial.print("keep track1 F: ");
            Serial.print(distF);
            Serial.print(" B: ");
            Serial.print(distB);
            Serial.print(" - ");
            Serial.println(passTrackNumber);
#endif
            passTrack1[passTrackNumber] = distF;
            passTrack1[passTrackNumber + 1] = distB;
            passTrackNumber = passTrackNumber + 2;
            lastEchoTime = millis();
          }
        }
        break;
      }
    case 0x84:   // got end of path
      {
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.println(passMonitorStepID, HEX);
#endif
        StopMonitorInterrupt();
        InterruptMoveAcrossPath(0x00);
        //      InterruptMove(uint8_t action, uint8_t retCode)
        bitWrite(passMonitorStepID, passMonitorInterruptBit, 0); // clear interrupt flag
        bitWrite(passMonitorStepID, passMonitorRequestBit, 0);
        passMonitorStepID++;
        //       passTrackLeftHoles[3] = Wheels.GetCurrentHolesCount(leftWheelId);
        //       passTrackRightHoles[3] = Wheels.GetCurrentHolesCount(rightWheelId);
        break;
      }
    case 0x05:    // we are out of the pass
      {
        passTrackLeftHoles[3] = Wheels.GetCurrentHolesCount(leftWheelId);
        passTrackRightHoles[3] = Wheels.GetCurrentHolesCount(rightWheelId);
        passTrack1[2 * passNumberTrack1 - 2] = PingFront();
        delay(delayBetween2Ping); // to let time to refresh echos
        passTrack1[2 * passNumberTrack1 - 1] = PingBack();
#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
#endif
        EchoServoAlign(90, false);
        obstacleDetectionOn = true;
        bitWrite(passMonitorStepID, passMonitorInterruptBit, 0); // clear interrupt flag
        bitWrite(passMonitorStepID, passMonitorRequestBit, 0);
        passMonitorStepID++;
        break;
      }
    case 0x06:
      {
        /*
          we are most likely out the pass
        */
        delay(1000);

#if defined(debugAcrossPathOn)
        Serial.print("MoveAcrossPath: 0x");
        Serial.print(passMonitorStepID, HEX);
        Serial.print(" encodersToStop: ");
        Serial.print(encodersToStop, HEX);
        Serial.print(" Whl: ");
        Serial.print( Wheels.GetCurrentHolesCount(leftWheelId));
        Serial.print(" Whr: ");
        Serial.println( Wheels.GetCurrentHolesCount(rightWheelId));
        for (int i = 0; i < 5; i++)
        {
          Serial.print("Hl: ");
          Serial.print(passTrackLeftHoles[i]);
          Serial.print(" Hr: ");
          Serial.println(passTrackRightHoles[i]);
        }
        for (int i = 0; i < passNumberTrack1; i++)
        {
          Serial.print("echo F: ");
          Serial.print(passTrack1[2 * i]);
          Serial.print(" B: ");
          Serial.println(passTrack1[2 * i + 1]);
        }
#endif
        if (!encodersToStop) {
          StopMonitorInterrupt();
          passMonitorStepID ++;
          MoveForward(reqMove);
        }
        break;
      }
    case 0x07:
      {
        if (!bitRead(currentMove, toDoStraight))       // is move ended without getting end of path ?
        {
#if defined(debugAcrossPathOn)
          Serial.println("end move");
#endif
          passMonitorStepID = 0x2e;
        }
        break;
      }
    case 0x2e:
      {
        passTrackLeftHoles[4] = Wheels.GetCurrentHolesCount(leftWheelId);
        passTrackRightHoles[4] = Wheels.GetCurrentHolesCount(rightWheelId);
        EndMoveUpdate(moveAcrossPassEnded, 0x00);
        passMonitorStepID = passMonitorIddle;
        passInterruptBy = 0x00;
        obstacleDetectionOn = true;           // enable standard obstacle detection during move
        break;
      }
  }
}



void InterruptMoveAcrossPath(uint8_t retCode)
{
  /*
    if retCode==0x00 >> stop current moving and go to the next MoveAcrossPath step
    if retCode!=0x00 >> stop current moving and report move ended with retCode
  */
#if defined(debugAcrossPathOn)
  Serial.print("InterruptMoveAcrossPath: 0x");
  Serial.println(passMonitorStepID, HEX);
#endif
  StopMonitorInterrupt();
  bSpeedHigh[leftWheelId] = false;
  bSpeedHigh[rightWheelId] = false;
  WheelThresholdReached(leftWheelId);
  WheelThresholdReached(rightWheelId);
  passInterruptBy = 0x00;
  timePassMonitorStarted = millis();
  if (retCode != 0x00)
  {
    actStat = moveAcrossPassEnded;                                      // status move completed
    EndMoveUpdate(moveAcrossPassEnded, timeoutRetCode);                       // move not completed due to obstacle
    obstacleDetectionOn = true;
  }
  StopMotors();
  Wheels.StopWheelControl(true, true, 0, 0);
 // digitalWrite(encoderPower, LOW);
  
}

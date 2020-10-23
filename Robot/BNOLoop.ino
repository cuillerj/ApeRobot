/*
  BNOUpToDateFlag used to get regurarly alternativly BNO status or heading

  pendingPollingResp
    0x01 pending pooling request
    0x00 response has been received

  BNORequestedState: data contain to get or got state

  stepBNOInitLocation: define current step of the init localisation wich need 3 steps

*/
byte savBNORequestedState = BNOIddleState;

byte BNOLoop() {
  if (pendingPollingResp == 0x00 && millis() - lastBNOLoopTime < 75) { // to avoid going to fast
    return BNOLoopRetcode;
  }
  else {
    lastBNOLoopTime = millis();
  }
  if (pendingDataResp && pendingPollingResp == 0x00) {    // got polling response but not yet data
    if (millis() - pendingDataRespTimer < 1000) {  // to deal with timeout
      return BNOLoopRetcodeWaitingData;
    }
    else {
      Serial.println("timeout pendingDataResp:");
      pendingDataRespTimer = millis();
      pendingDataResp = false;
      return BNOLoopRetcodeDataTimeout;
    }
  }
  if (pendingPollingResp != 0x00) {
    if (millis() - pendingPollingRespTimer < 900) {  // to deal with polling timeout
      return BNOLoopRetcodeWaitingPoll;
    }
    else {
      Serial.print("timeout pendingPollingResp:");
      Serial.println(pendingPollingResp);
      pendingPollingRespTimer = millis();
      pendingPollingResp = 0x00;
      digitalWrite(RobotOutputRobotRequestPin, LOW);
      delay(1);
      return BNOLoopRetcodePollTimeout;
    }
  }
  else {
    pendingPollingRespTimer = millis();
  }
  if (bitRead(BNORequestedState, 0) && BNORequestedState != BNOGetStatus && BNORequestedState != BNOGetMode && BNORequestedState != BNOIddleState) {
    savBNORequestedState = BNORequestedState; // store the request to execute as soon as the subsystem is ready
  }
  if (expectedBNOMode != 0x00 && expectedBNOMode != BNOMode && millis() - lastSetModeTime > 5000) {
    //    requestChangeMonitorModeTime = millis();

    if (expectedBNOMode == MODE_IMUPLUS)
    {
      SetBNOMode(MODE_IMUPLUS);
    }
    if (expectedBNOMode == MODE_COMPASS)
    {
      SetBNOMode(MODE_COMPASS);
    }
    BNORequestedState = BNOGetMode;
    return BNOLoopRetcodeReady ;

  }

  if (expectedSubsystemStatus != monitSubsystemStatus && expectedSubsystemStatus != 0x00 && millis() - requestChangeMonitorModeTime > 1200) { // does the subsystem in the right status
    requestChangeMonitorModeTime = millis();

#if defined(debugGyroscopeOn)
    Serial.println("status");
#endif
    if (expectedSubsystemStatus != monitSubsystemStatus) { // is the subsystem in the right status
      if (!bitRead(expectedSubsystemStatus, monitGyroStatusBit) && bitRead(monitSubsystemStatus, monitGyroStatusBit))
      {
        GyroStopInitMonitor();
#if defined(debugGyroscopeOn)
        Serial.println(">GyroStop ");
#endif
      }
      else if (!bitRead(expectedSubsystemStatus, monitMagnetoStatusBit) && bitRead(monitSubsystemStatus, monitMagnetoStatusBit))
      {
        StopMagneto();
#if defined(debugGyroscopeOn)
        Serial.println("MagnetoStop");
#endif
      }
      else if ((bitRead(expectedSubsystemStatus, monitGyroStatusBit) && !bitRead(monitSubsystemStatus, monitGyroStatusBit)) || ((bitRead(expectedSubsystemStatus, monitGyroforward) != bitRead(monitSubsystemStatus, monitGyroforward))))
      {
        #if defined(debugGyroscopeOn)
        Serial.print(">GyroStart:");
        Serial.println(bitRead(monitSubsystemStatus, monitGyroforward));
        Serial.println(expectedSubsystemStatus, HEX);
        Serial.println(monitSubsystemStatus, HEX);
#endif
        bitWrite(monitSubsystemStatus, monitGyroforward, bitRead(expectedSubsystemStatus, monitGyroforward)) ;
        GyroStartInitMonitor(bitRead(monitSubsystemStatus, monitGyroforward));

      }

      else if (bitRead(expectedSubsystemStatus, monitMagnetoStatusBit) && !bitRead(monitSubsystemStatus, monitMagnetoStatusBit))
      {
        StartMagneto();
#if defined(debugGyroscopeOn)
        Serial.println(">MagnetoStart");
#endif
      }
    }
    BNORequestedState = BNOGetStatus ;
    return BNOLoopRetcodeReady ;
  }

  if (stepBNOInitLocation != 0) { // reentering 3 times for a complete location initialization
    InitBNOLocation();
    return BNOLoopRetcodeWaitInitLocation;
  }

  switch (BNORequestedState) {

    case BNOGetHeadingState:
#if defined(debugGyroscopeOn)
      Serial.println("GetBNOHeadingLocation");
#endif
      GetBNOHeadingLocation();
      return BNOLoopRetcodeReady;
    case BNOGotHeadingState:
      if (BNORequestSequence) {
        BNORequestedState = BNOGetRightState;
      }
      else {
        BNORequestedState = BNOGetLeftState;
      }
      return BNOLoopRetcodeReady  ;

    case BNOGetRightState:
#if defined(debugGyroscopeOn)
      Serial.println("GetBNORightLocation");
#endif
      GetBNORightLocation();
      return BNOLoopRetcodeReady;
    case BNOGotRightState:
      if (BNORequestSequence) {
        BNORequestedState = BNOGetLeftState;
      }
      else {
        BNORequestedState = BNOGetLeftState;
      }
      return BNOLoopRetcodeReady;

    case BNOGetLeftState:
#if defined(debugGyroscopeOn)
      Serial.println("GetBNOLeftLocation");
#endif
      GetBNOLeftLocation();
      return BNOLoopRetcodeReady;
    case BNOGotLeftState:
      BNORequestedState = BNOIddleState ;
      return BNOLoopRetcodeReady;

    case BNOUpdateHoles:
#if defined(debugGyroscopeOn)
      Serial.println("BNOUpdatedHoles");
#endif
      UpdateBNOMove();
      BNORequestedState = BNOUpdatedHoles;
      return BNOLoopRetcodeReady;
    case BNOUpdatedHoles:
      if (BNORequestSequence) {
        BNORequestedState = BNOGetHeadingState;
      }
      else {
        BNORequestedState = BNOGetLeftState;
      }
      return BNOLoopRetcodeReady;

    case BNOGetRelativeHeadingState:
#if defined(debugGyroscopeOn)
      Serial.println("BNOGetRelativeHeadingState");
#endif
      GyroGetHeadingRegisters();
      return BNOLoopRetcodeReady;
    case BNOGotRelativeHeadingState:
      gyroUpToDate = 0x02;
      BNORequestedState = BNOIddleState   ;
      return BNOLoopRetcodeReady;

    case BNOGetAbsoluteHeadingState:
#if defined(debugGyroscopeOn)
      Serial.println("BNOGetAbsoluteHeadingState");
#endif
      GetAbsoluteHeading();
      return BNOLoopRetcodeReady;
    case BNOGotAbsoluteHeadingState:
      BNORequestedState = BNOIddleState  ;
      return BNOLoopRetcodeReady;

    case BNOGetCompasHeadingState:
#if defined(debugGyroscopeOn)
      Serial.println("BNOGetCompasHeadingState");
#endif
      GetNorthOrientation();
      return BNOLoopRetcodeReady;
    case BNOGotCompasHeadingState:
      BNORequestedState = BNOIddleState  ;
      return BNOLoopRetcodeReady;

    case BNOGetStatus:
      RequestSystemStatus();
      return BNOLoopRetcodeReady ;
    case BNOGotStatus:
#if defined(debugGyroscopeOn)
      Serial.print("got status savstate:0x");
      Serial.println(savBNORequestedState, HEX);
#endif
      if (savBNORequestedState == BNOIddleState) {
        BNORequestedState = BNOIddleState;
      }
      else if (expectedSubsystemStatus == monitSubsystemStatus && expectedSubsystemStatus != 0x0 && expectedBNOMode != 0x00 && expectedBNOMode == BNOMode ) {  // subsystem ready for the request
        BNORequestedState = savBNORequestedState;
        savBNORequestedState = BNOIddleState;
      }
      else {
        BNORequestedState = BNOIddleState;
      }
      return BNOLoopRetcodeReady;

    case BNOGetMode:
      GetBNO055Status();
      return BNOLoopRetcodeReady;
    case BNOGotMode:
#if defined(debugGyroscopeOn)
      Serial.print("got mode savstate:0x");
      Serial.println(savBNORequestedState, HEX);
#endif
      if (savBNORequestedState == BNOIddleState) {
        BNORequestedState = BNOIddleState;
      }
      else if (expectedSubsystemStatus == monitSubsystemStatus && expectedSubsystemStatus != 0x0 && expectedBNOMode != 0x00 && expectedBNOMode == BNOMode ) {  // subsystem ready for the request
        BNORequestedState = savBNORequestedState;
        savBNORequestedState = BNOIddleState;
      }
      else {
        BNORequestedState = BNOIddleState;
      }
      return BNOLoopRetcodeReady;


  }

  if (rebootPhase == 0x00 && millis() - iddleTimer > 60000 && toDo == 0x00 && toDoDetail == 0x00  && !bitRead(appTrace, traceBitNO))
  {
    if (BNOMode != MODE_IMUPLUS)
    {
      //      SetBNOMode(MODE_IMUPLUS);
      expectedBNOMode = MODE_IMUPLUS;
      return BNOLoopRetcodeReady;
    }
  }
  if (millis() - timeGyroRotation > delayGyroRotation && bitRead(toDo, toDoRotation) == 1 && bitRead(toDoDetail, toDoGyroRotation) == 1 && !leftMotor.RunningMotor() && !rightMotor.RunningMotor()) //
  {
    if (BNOMode != MODE_IMUPLUS )
    {
      //      SetBNOMode(MODE_IMUPLUS);
      expectedBNOMode = MODE_IMUPLUS;
    }
    else if (!bitRead(monitSubsystemStatus, monitGyroStatusBit)) {
      bitWrite(expectedSubsystemStatus, monitMagnetoStatusBit, 1);
    }
    else {
      timeGyroRotation = millis();
#if defined(debugLoop)
      Serial.print("loop gyro todo 0x:");
      Serial.print(toDo, HEX);
      Serial.print(" loop gyro tododetail 0x:");
      Serial.println(toDoDetail, HEX);
#endif
      if (gyroRotationRetry >= maxGyroRotationRetry)
      {

#if defined(debugGyroscopeOn)
        Serial.print("gyro to many retry:0x");
        Serial.print(toDo, HEX);
        Serial.print(" loop gyro tododetail 0x:");
        Serial.println(toDoDetail, HEX);
#endif
        bitWrite(diagRobot, diagRobotGyroRotation, 1);
        bitWrite(toDoDetail, toDoGyroRotation, 0);
        EndMoveUpdate(moveEnded, rotationKoToManyRetry);
      }
      else
      {
        if (gyroUpToDate == 0x00)
        {
#if defined(debugGyroscopeOn)
          Serial.print("request heading: 0x");
          Serial.print(toDo, HEX);
          Serial.print(" dodetail 0x:");
          Serial.println(toDoDetail, HEX);
#endif
          // GyroGetHeadingRegisters();

          Serial.print(" set to:");

          BNORequestedState = BNOGetRelativeHeadingState;
          gyroUpToDate = 0x01;
        }
        if (gyroUpToDate == 0x02)
        {
#if defined(debugGyroscopeOn)
          Serial.print("got heading 0x:");
          Serial.print(toDo, HEX);
          Serial.print(" tododetail 0x:");
          Serial.println(toDoDetail, HEX);
#endif
          gyroUpToDate = 0x00;
          int retCode = GyroscopeRotate();
#if defined(debugGyroscopeOn)
          Serial.print("back gyrorotate 0x:");
          Serial.print(toDo, HEX);
          Serial.print(" 0x:");
          Serial.print(toDoDetail, HEX);
          Serial.print(" retCode:");
          Serial.println(retCode);
#endif
          if (retCode != -1 && abs(retCode) > 1)
          {
            ComputeAngleAndPositionVSGyro(gyroscopeHeading[gyroscopeHeadingIdx]);
            bitWrite(toDoDetail, toDoGyroRotation, 0);
            bitWrite(diagRobot, diagRobotGyroRotation, 1);
            EndMoveUpdate(moveEnded, retCode);
          }
        }
      }
    }
    return BNOLoopRetcodeReady;
  }

  if ((millis() - lastUpdateBNOMoveTime > delayBetweenlastUpdateBNOMoveTime && bitRead(currentMove, toDoStraight) == true ))
  {
    //    UpdateBNOMove();
    BNORequestedState = BNOUpdateHoles;
  }

  return BNOLoopRetcode;

}

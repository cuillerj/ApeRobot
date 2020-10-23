void CheckMoveSynchronisation()       // check that the 2 wheels are rotating at the same pace
{
  long currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  long currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);

  float deltaMove = abs(int(currentLeftHoles) - currentRightHoles);
  bitWrite(diagMotor, diagMotorPbSynchro, 0);       // position bit diagMotor
  bitWrite(diagMotor, diagMotorPbLeft, 0);       // position bit diagMotor
  bitWrite(diagMotor, diagMotorPbRight, 0);       // position bit diagMotor

  boolean pbWheelStopped = false;
#if defined(debugMotorsOn)
  Serial.print("check move synch prevleft: ");
  Serial.print(prevCheckLeftHoles);
  Serial.print(" currentleft: ");
  Serial.print(currentLeftHoles);
  Serial.print(" prevright: ");
  Serial.print(prevCheckRightHoles);
  Serial.print(" currentright: ");
  Serial.print(currentRightHoles);
  Serial.print(" delta: ");
  Serial.print(deltaMove);
  Serial.print(" currMove:0b");
  Serial.print(currentMove, BIN);
  Serial.print( " equalSpeed:");
  Serial.println(equalSpeed);
#endif
  if (bitRead(toDo, toDoRotation) || bitRead(toDo, toDoAlign))
  {
    if ((currentLeftHoles == 0 || currentLeftHoles == prevCheckLeftHoles) && (currentRightHoles == 0 || currentRightHoles == prevCheckRightHoles))
    {
#if defined(debugMotorsOn)
      Serial.println("pb synchro rotation");
#endif
      pbSynchro = true;
      //    bitWrite(diagMotor, diagMotorPbLeft, 1);       // position bit diagMotor
    }
  }
  else {
    if (currentLeftHoles == 0 || currentLeftHoles == prevCheckLeftHoles)
    {
#if defined(debugMotorsOn)
      Serial.println("pb left ");
#endif
      pbWheelStopped = true;
      bitWrite(diagMotor, diagMotorPbLeft, 1);       // position bit diagMotor
    }
    if (currentRightHoles == 0 || currentRightHoles == prevCheckRightHoles)
    {
#if defined(debugMotorsOn)
      Serial.println("pb right ");
#endif
      pbWheelStopped = true;
      bitWrite(diagMotor, diagMotorPbRight, 1);       // position bit diagMotor
    }
  }
  if ( equalSpeed && (deltaMove > 10 && deltaMove / float(currentLeftHoles)   > 0.2)  )
  {
#if defined(debugMotorsOn)
    Serial.println("pb delta");
#endif
    pbSynchro = true;
  }

//  int deltaHeading = gyroscopeHeading[gyroscopeHeadingIdx];

  /*
    int deltaH = currentLeftHoles - currentRightHoles;
    if ( PIDMode && equalSpeed && deltaH != 0  && currentLeftHoles > leftWheelEncoderHoles && currentRightHoles > rightWheelEncoderHoles )
    {
    float avgH = (currentLeftHoles + currentRightHoles) / 2;
    leftSetpoint = leftSetpoint - (leftSetpoint * deltaH) / (2 * avgH);
    //    leftSetpoint = max(outLimit[leftMinOut], leftSetpoint);
    //    leftSetpoint = min(outLimit[leftMaxOut], leftSetpoint);
    rightSetpoint = rightSetpoint + (rightSetpoint * deltaH) / (2 * avgH);
    //   rightSetpoint = max(outLimit[rightMinOut], rightSetpoint);
    //   rightSetpoint = min(outLimit[rightMaxOut], rightSetpoint);
    #if defined(debugMotorsOn)
    Serial.print("> speed left:");
    Serial.print(leftInput);
    Serial.print(" right:");
    Serial.print(rightInput);
    #endif
    }
  */
  if (pbWheelStopped == true || pbSynchro == true)
  {
#if defined(debugMotorsOn)
    Serial.println("stop wheels ");
#endif
    StopMotors();
    Horn(true, 750);

    /*
      GyroGetHeadingRegisters();interruptmov
      StopEchoInterrupt(true, true);                    // stop obstacles detection
      Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
      digitalWrite(encoderPower, LOW);
      detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
      bitWrite(diagMotor, diagMotorPbSynchro, 1);       // position bit diagMotor
      pinMode(wheelPinInterrupt, INPUT);
    */
#if defined(debugMotorsOn)
    Serial.print("move synchro pb deltaMove: ");
    Serial.print(deltaMove);
    Serial.print(" ");
    Serial.println(deltaMove / float(currentLeftHoles));
#endif

    if (bitRead(currentMove, toDoRotation) == true)
    {
      // ComputeNewLocalization(0x01);
    }
    else {
      //  ComputeNewLocalization(0xff);
    }
    if (pbSynchro)
    {
      InterruptMove(moveEnded, moveWheelSpeedInconsistancy);
    }
    else {
      InterruptMove(moveEnded, moveKoDueToWheelStopped);
    }

#if defined(debugMotorsOn)
    Serial.print("min level left: ");
    Serial.print(Wheels.GetMinLevel(leftWheelId));
    Serial.print(" max: ");
    Serial.println(Wheels.GetMaxLevel(leftWheelId));
    Serial.print("min level right: ");
    Serial.print(Wheels.GetMinLevel(rightWheelId));
    Serial.print(" max: ");
    Serial.println(Wheels.GetMaxLevel(rightWheelId));
#endif
  }
  prevCheckLeftHoles = currentLeftHoles;
  prevCheckRightHoles = currentRightHoles;
}

/*

*/
/*
  void ExecuteRotation(int rotation)
  {
  if (abs(rotation) > 3 * maxInertialRotation)
  {
    if (rotation >= 0)
    {
      int newRotation = rotation - maxInertialRotation;
      Rotate(newRotation, check);
  #if defined(debugMoveOn)
      Serial.print("rot1:");
      Serial.println( newRotation);
      Serial.print(" delay ");
  #endif
    }
    else
    {
      int newRotation = rotation + maxInertialRotation;
      Rotate(newRotation, check);
  #if defined(debugMoveOn)
      Serial.print("rot2:");
      Serial.println( rotation + maxInertialRotation);
  #endif
    }
  }
  else
  {
    if (rotation >= 0)
    {
      int newRotation = max(round(rotation * 0.8), minRotEncoderAbility);
      Rotate(newRotation, check);
  #if defined(debugMoveOn)
      Serial.print("rot3:");
      Serial.println( newRotation);
  #endif
    }
    else
    {
      int newRotation = min(round(rotation * 0.8), -minRotEncoderAbility);
      Rotate(newRotation, check);
  #if defined(debugMoveOn)
      Serial.print("rot4:");
      Serial.println(newRotation);
  #endif
    }
  }
  }
*/
void Rotate( int orientation, boolean check) {
  wheelCheckDelay = wheelCheckRotateDelay;
  leftSetpoint = optimalLowStraightSpeed;
  rightSetpoint = leftSetpoint;
  if ( bitRead(pendingAction, pendingLeftMotor) == false && bitRead(pendingAction, pendingRightMotor) == false );
  {
    boolean availableRotation = true;
    if (check)
    {
      availableRotation = CheckRotationAvaibility(orientation);
    }

    if (availableRotation)    // check rotation
    {
      currentMove = 0x00;
      saveCurrentMove = 0x00;
      bitWrite(currentMove, toDoRotation, true);
      bitWrite(saveCurrentMove, toDoRotation, true);
      if (!bitRead(toDoDetail, toDoGyroRotation))
      {
        bitWrite(toDo, toDoRotation, false);       // position bit toDo move
      }
      deltaPosX = 0;
      deltaPosY = 0;
      unsigned int lentghLeftToDo = 0;
      unsigned int lentghRightToDo = 0;
      if (orientation != 0)
      {
        Serial.print("rotationToDo:");
        Serial.println(orientation);
        // Rotate
        lentghLeftToDo = RotationCalculation(abs(orientation));
        lentghRightToDo = lentghLeftToDo;
        ComputerMotorsRevolutionsAndrpm(lentghLeftToDo, leftMotorPWM * leftRotatePWMRatio, lentghRightToDo, rightMotorPWM * rightRotatePWMRatio);
        if (orientation > 0)
        {
          bLeftClockwise = true;
          bitWrite(currentMove, toDoClockwise, true);
          bitWrite(saveCurrentMove, toDoClockwise, true);
        }
        else {
          bLeftClockwise = false;
          bitWrite(currentMove, toDoClockwise, false);
          bitWrite(saveCurrentMove, toDoClockwise, false);
        }
        bRightClockwise = bLeftClockwise;
        startMotors();
      }
      Serial.println("rotate ");
    }
    else {
      EndMoveUpdate(moveEnded, moveKoDueToNotEnoughSpace);
      toDo = 0x00;
      toDoDetail = 0x00;
    }
  }
}
void pulseMotors(unsigned int pulseNumber)
{
  if (bitRead(monitSubsystemStatus, monitGyroStatusBit)) {
    //GyroGetHeadingRegisters();
    BNORequestedState = BNOGetRelativeHeadingState;
  }
  if (bitRead(monitSubsystemStatus, monitMagnetoStatusBit)) {
    BNORequestedState = BNOGetCompasHeadingState;
  }
  // bitWrite(pendingAction, pendingLeftMotor, true);
  //  bitWrite(pendingAction, pendingRightMotor, true);
#if defined(debugMotorsOn)
  Serial.print("reqSpeed: ");
  Serial.print(leftMotorPWM);
  Serial.print(" ");
  Serial.println(rightMotorPWM);
#endif
  diagMotor = 0x00;
  //  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, FALLING);
  Wheels.StartWheelPulse(pulseNumber);
  leftMotor.RunMotor(bLeftClockwise,  leftMotorPWM);
  rightMotor.RunMotor(bRightClockwise,  rightMotorPWM);
  timeMotorStarted = millis();
}
float  RotationCalculation(int orientation) {
  float distToDo = round(((iRobotWidth * PI / 360) * orientation));
  Serial.print("distRot: ");
  Serial.println(distToDo);
  return (distToDo);
}

void WheelInterrupt()   // wheel controler set a software interruption due to threshold reaching
{
 // detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  wheelIdInterruption = Wheels.GetLastWheelInterruptId();  // get which wheel Id reached the threshold to be analysed in the next loop
  if (wheelIdInterruption != 5 && bSpeedHigh[wheelIdInterruption] == false)    //
  {
    Wheels.ClearThreshold(wheelIdInterruption);                      // clear the threshold flag to avoid any more interruption
  }
  else{
        StopMotors();
  }
  //WheelThresholdReached(wheelIdInterruption);                      // call the threshold analyse
  PIDMode = false;
}

void WheelThresholdReached(uint8_t wheelId)
{
  if (wheelId != 5)
  {
    if (wheelId == leftWheelId)
    {
      leftMotor.AdjustMotorPWM(0);                             // stop firstly the motor that reached the threshold
    }
    else
    {
      rightMotor.AdjustMotorPWM(0);                         // stop firstly the motor that reached the threshold
    }
    leftSetpoint=0;
    rightSetpoint=0;
    StopMotors();
    timeMotorStarted = 0;
    if (passMonitorStepID != passMonitorIddle)
    {
      passInterruptBy = 0x02;
      //   MoveAcrossPath();
    }
    else {
   //   StopEchoInterrupt();                    // stop obstacles detection
    }
    timeAfterStopMotors = millis();
    bitWrite(pendingAction, pendingLeftMotor, false);     // clear the flag pending action motor
    bitWrite(pendingAction, pendingRightMotor, false);    // clear the flag pending action motor
    if ( bitRead(currentMove, toDoStraight) == true)      // robot was moving straight
    {
  //    attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, FALLING);   // to keep track of move dur to inertia
      encodersToStop = true;                               // to keep wheel ecoders running a little bit
    }
    if ( bitRead(currentMove, toDoRotation) == true)      // robot was turning around
    {
      delay(500);                                      // wait a little for robot intertia
      Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
      detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
      digitalWrite(encoderPower, LOW);
      gyroUpToDate = 0x00;
   //   GyroGetHeadingRegisters();
      timeAfterStopMotors = millis();
      bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation
      delay(100);
      if (bitRead(toDo, toDoStraight) == false && bitRead(toDoDetail, toDoGyroRotation) == 0 && !bitRead(toDoDetail, toDoAlignRotate))
      {
        if (pbSynchro) {
          EndMoveUpdate(moveEnded, moveWheelSpeedInconsistancy);
        }
        else
        {
          EndMoveUpdate(moveEnded, 0x00);
        }
      }
      //   bitWrite(currentMove, toDoRotation, false) ;        // clear flag todo rotation


    }
  }
  else                      // wheel mode pulse
  {
    StopMotors();
    //  GyroGetHeadingRegisters();
  }
  Horn(true, 100);
}

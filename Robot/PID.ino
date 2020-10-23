void TurnPIDOn()
{
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  headingPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(outLimit[leftMinOut], outLimit[leftMaxOut]);
  rightPID.SetOutputLimits(outLimit[rightMinOut], outLimit[rightMaxOut]);
  headingPID.SetOutputLimits(-10., 10.);
  SetPIDKx();
  SetPIDSampleTime(delayMiniBetweenHoles * 3);
#if defined(debugPID)
  {
    Serial.print("leftMinOut;");
    Serial.print(outLimit[leftMinOut]);
    Serial.print(" leftMaxOut;");
    Serial.println(outLimit[leftMaxOut]);
    Serial.print("rightMinOut;");
    Serial.print(outLimit[rightMinOut]);
    Serial.print(" righMaxOut;");
    Serial.println(outLimit[rightMaxOut]);
  }
#endif
}
void SetPIDKx()
{
  leftPID.SetTunings(KxLeft[KpRegister], KxLeft[KiRegister], KxLeft[KdRegister]);
  rightPID.SetTunings(KxRight[KpRegister], KxRight[KiRegister], KxRight[KdRegister]);
  headingPID.SetTunings(KxHeading[KpRegister], KxHeading[KiRegister], KxHeading[KdRegister]);
  // leftPID.SetSampleTime((delayMiniBetweenHoles / 3));
  // rightPID.SetSampleTime((delayMiniBetweenHoles / 3));
#if defined(debugPID)
  {
    Serial.print("left Kp:");
    Serial.print(leftPID.GetKp());
    Serial.print(" Ki:");
    Serial.print(leftPID.GetKi());
    Serial.print(" Kd:");
    Serial.println(leftPID.GetKd());
    Serial.print("right Kp:");
    Serial.print(rightPID.GetKp());
    Serial.print(" Ki:");
    Serial.print(rightPID.GetKi());
    Serial.print(" Kd:");
    Serial.println(rightPID.GetKd());
    Serial.print("leftMinOut;");
    Serial.println(outLimit[leftMinOut]);
    Serial.print("heading Kp:");
    Serial.print(headingPID.GetKp());
    Serial.print(" Ki:");
    Serial.print(headingPID.GetKi());
    Serial.print(" Kd:");
    Serial.println(headingPID.GetKd());
  }
#endif
}
void ComputePID()
{
  // double speedLeft = Wheels.GetLastTurnSpeed(leftWheelId) * 100;
  // speedLeft = (Wheels.Get2LastTurnSpeed(leftWheelId) * 100 + speedLeft) / 2;

  int refHeading = 0;
  float leftSpeed = 0;
  float rightSpeed = 0;
  if( BNOLocationHeading == -1){
    return;
  }
  if (millis() - timePID > 150) // update speed every 1/4 turn
  {
    timePID = millis();
    if (PIDFirstLoop && BNOLocationHeading != -1) {
      refHeading = BNOLocationHeading;
      PIDFirstLoop = false;
#if defined(debugPID)
      Serial.print("refHeading:");
      Serial.println(refHeading);
#endif
    }
    int deltaHeading = ComputeAngleDiff( int(BNOLocationHeading), refHeading);
    if (abs(deltaHeading) > 1) {
      if (abs(deltaHeading < 45)) {
        headingInput = deltaHeading;
        headingPID.Compute();
        int sign = 1;
        if (bitRead(currentMove, toDoBackward) || bitRead(toDo, toDoBackward)) {
          sign = -1;
        }
        leftSetpoint = max(minRPS100, min(maxRPS100, (leftSetpoint -  sign * (leftSetpoint * headingOutput) / 180)));
        rightSetpoint = max(minRPS100, min(maxRPS100, (rightSetpoint  +  sign * (rightSetpoint * headingOutput) / 180)));
      }
      else {
        Serial.print(" pb heading:");
        Serial.println(deltaHeading);
      }
#if defined(debugPID)
      Serial.print(deltaHeading);
      Serial.print("-");
      Serial.print(headingInput);
      Serial.print(" HOU:");
      Serial.print(headingOutput);
      Serial.print(" LsetP:");
      Serial.print(leftSetpoint);
      Serial.print(" RsetP:");
      Serial.println(rightSetpoint);
#endif
    }
    leftSpeed = Wheels.GetTurnSpeed(leftWheelId) * 100;
    rightSpeed = Wheels.GetTurnSpeed(rightWheelId) * 100;
    leftInput = leftSpeed;
    rightInput = rightSpeed;
    leftPID.Compute();
    rightPID.Compute();
    leftMotor.AdjustMotorPWM(leftOutput);
    rightMotor.AdjustMotorPWM(rightOutput);
#if defined(debugPID)
    {
      Serial.print("leftIn:");
      Serial.print(leftSpeed);
      Serial.print(" leftOU:");
      Serial.print(leftOutput);
      Serial.print(" rightIn:");
      Serial.print(rightSpeed);
      Serial.print(" rightOU:");
      Serial.print(rightOutput);
      Serial.print(" LsetP:");
      Serial.print(leftSetpoint);
      Serial.print(" RsetP:");
      Serial.println(rightSetpoint);
    }
#endif
  }

}
void SetPIDSampleTime(double sampTime)
{
  leftPID.SetSampleTime(sampTime);
  rightPID.SetSampleTime(sampTime);
  headingPID.SetSampleTime(sampTime);
}

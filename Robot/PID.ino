void TurnPIDOn()
{

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(outLimit[leftMinOut], outLimit[leftMaxOut]);
  rightPID.SetOutputLimits(outLimit[rightMinOut], outLimit[rightMaxOut]);
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
    Serial.print(outLimit[leftMinOut]);
  }
#endif
}
void ComputePID()
{
  // double speedLeft = Wheels.GetLastTurnSpeed(leftWheelId) * 100;
  // speedLeft = (Wheels.Get2LastTurnSpeed(leftWheelId) * 100 + speedLeft) / 2;
  if (millis() >= timePID + 25000. / optimalHighStraightSpeed ) // update speed every 1/4 turn
  {
    leftInput = Wheels.GetTurnSpeed(leftWheelId) * 100;
    rightInput = Wheels.GetTurnSpeed(rightWheelId) * 100;
  }
  //  double speedLeft = Wheels.GetTurnSpeed(leftWheelId) * 100;
  // speedLeft = (Wheels.Get2LastTurnSpeed(leftWheelId) * 100 + speedLeft) / 2;
  // leftInput = speedLeft;
  leftPID.Compute();
  rightPID.Compute();
  leftMotor.AdjustMotorPWM(leftOutput);
  rightMotor.AdjustMotorPWM(rightOutput);
  // double speedRight = Wheels.GetLastTurnSpeed(rightWheelId) * 100;
  // speedRight = (Wheels.Get2LastTurnSpeed(rightWheelId) * 100 + speedRight) / 2;
  //double speedRight = Wheels.GetTurnSpeed(rightWheelId) * 100;
  // rightInput = speedRight;


#if defined(debugPID)
  {
    if (millis() >= timePID + optimalHighStraightSpeed * maxWheelEncoderHoles / 200)
    {
      Serial.print("leftIn:");
      Serial.print(leftInput);
      Serial.print(" leftOU:");
      Serial.print(leftOutput);
      Serial.print(" rightIn:");
      Serial.print(rightInput);
      Serial.print(" rightOU:");
      Serial.print(rightOutput);
      Serial.print(" LsetP:");
      Serial.print(leftSetpoint);
      Serial.print(" setP:");
      Serial.println(rightSetpoint);
    }
  }
#endif
  timePID = millis();
}
void SetPIDSampleTime(double sampTime)
{
  leftPID.SetSampleTime(sampTime);
  rightPID.SetSampleTime(sampTime);
}

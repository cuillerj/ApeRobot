void TurnPIDOn()
{

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(outLimit[leftMinOut], outLimit[leftMaxOut]);
  rightPID.SetOutputLimits(outLimit[rightMinOut], outLimit[rightMaxOut]);
  SetPIDKx();
  SetPIDSampleTime(delayMiniBetweenHoles / 3);
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
  leftPID.SetTunings(Kx[KpRegister], Kx[KiRegister], Kx[KdRegister]);
  rightPID.SetTunings(Kx[KpRegister], Kx[KiRegister], Kx[KdRegister]);
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
  double speedLeft = Wheels.GetLastTurnSpeed(leftWheelId) * 100;
  speedLeft = (Wheels.Get2LastTurnSpeed(leftWheelId) * 100 + speedLeft) / 2;
  leftInput = speedLeft;
  leftPID.Compute();
  leftMotor.AdjustMotorPWM(leftOutput);
  double speedRight = Wheels.GetLastTurnSpeed(rightWheelId) * 100;
  speedRight = (Wheels.Get2LastTurnSpeed(rightWheelId) * 100 + speedRight) / 2;
  rightInput = speedRight;
  rightPID.Compute();
  rightMotor.AdjustMotorPWM(rightOutput);
  timePID = millis();
#if defined(debugPID)
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
#endif
}
void SetPIDSampleTime(double sampTime)
{
  leftPID.SetSampleTime(sampTime);
  rightPID.SetSampleTime(sampTime);
}


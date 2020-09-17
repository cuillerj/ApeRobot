void CalibrateWheels()           // calibrate encoder levels and leftToRightDynamicAdjustRatio according to mesurments
{
  saveLeftWheelInterrupt = 0;
  saveRightWheelInterrupt = 0;
  digitalWrite(encoderPower, HIGH);
  //  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, RISING);
  Wheels.StartWheelControl(true, false, 0 , true, false, 0 , false, false, 0, false, false, 0);
  leftMotor.RunMotor(0,  leftMotorPWM );
  rightMotor.RunMotor(1, rightMotorPWM * leftToRightDynamicAdjustRatio);
  delay(4000);                                    // run motors 4 seconds
  rightMotor.StopMotor();                         // stop firstly the motor that reached the threshold
  leftMotor.StopMotor();                          // stop the other motor to avoid to turn round
  StopEchoInterrupt();                    // stop obstacles detection
  delay(200);                                       // wait a little for robot intertia
  GyroGetHeadingRegisters();
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  digitalWrite(encoderPower, LOW);
  detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  //  pinMode(wheelPinInterrupt, INPUT);
  leftWheeelCumulative = leftWheeelCumulative + Wheels.GetCurrentHolesCount(leftWheelId);
  rightWheeelCumulative = rightWheeelCumulative + Wheels.GetCurrentHolesCount(rightWheelId);
  unsigned int minLeftLevel = Wheels.GetMinLevel(leftWheelId);
  unsigned int maxLeftLevel = Wheels.GetMaxLevel(leftWheelId);
  unsigned int minRightLevel = Wheels.GetMinLevel(rightWheelId);
  unsigned int maxRightLevel = Wheels.GetMaxLevel(rightWheelId);
  unsigned int leftAvgLevel = (maxLeftLevel + minLeftLevel) / 2;
  unsigned int leftDiffLevel = (maxLeftLevel - minLeftLevel) / 4;
  unsigned int leftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int rightHoles = Wheels.GetCurrentHolesCount(rightWheelId);
  leftIncoderLowValue = leftAvgLevel - leftDiffLevel;
  leftIncoderHighValue = leftAvgLevel + leftDiffLevel;
  unsigned int rightAvgLevel = (maxRightLevel + minRightLevel) / 2;
  unsigned int rightDiffLevel = (maxRightLevel - minRightLevel) / 4;
  rightIncoderLowValue = rightAvgLevel - rightDiffLevel;
  rightIncoderHighValue = rightAvgLevel + rightDiffLevel;
  Serial.print(leftWheeelCumulative - rightWheeelCumulative);
  if (leftHoles != rightHoles)
  {
    Serial.print (" > diff ");
    if ((leftWheeelCumulative - rightWheeelCumulative > 0) && (leftHoles > rightHoles) )
    {
      Serial.println(" + ");
      leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio + 0.1;
    }
    if (( rightWheeelCumulative - leftWheeelCumulative > 0) && ( rightHoles > leftHoles) )
    {
      Serial.println(" - ");
      leftToRightDynamicAdjustRatio = leftToRightDynamicAdjustRatio - 0.1;
    }
  }
#if defined(debugWheelControlOn)
  Serial.print("min level left: ");
  Serial.print(minLeftLevel);
  Serial.print(" max: ");
  Serial.println(maxLeftLevel);
  Serial.print("min level right");
  Serial.print(minRightLevel);
  Serial.print(" max: ");
  Serial.println(maxRightLevel);
  Serial.print("Holes left: ");
  Serial.print(leftHoles);
  Serial.print(" right: ");
  Serial.println(rightHoles);
  Serial.print("speed left  ");
  Serial.print(Wheels.GetLastTurnSpeed(leftWheelId) * 60);
  Serial.print(" ");
  Serial.print(Wheels.Get2LastTurnSpeed(leftWheelId) * 60);
  Serial.print(" right  ");
  Serial.print(Wheels.GetLastTurnSpeed(rightWheelId) * 60);
  Serial.print(" ");
  Serial.println(Wheels.Get2LastTurnSpeed(rightWheelId) * 60);
  Serial.print("Cumulative Holes left: ");
  Serial.print(leftWheeelCumulative);
  Serial.print(" right: ");
  Serial.println(rightWheeelCumulative);
  Serial.print("dynamic adjust: ");
  Serial.println(leftToRightDynamicAdjustRatio);
  Serial.print("encoder left low: ");
  Serial.print(leftIncoderLowValue);
  Serial.print(" high: ");
  Serial.println(leftIncoderHighValue);
  Serial.print("encoder right low: ");
  Serial.print(rightIncoderLowValue);
  Serial.print(" high: ");
  Serial.println(rightIncoderHighValue);
#endif
  SendEncoderMotorValue();
}

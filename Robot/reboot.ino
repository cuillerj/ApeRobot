void CheckEndOfReboot()  //
{

  if (rebootPhase == 6) // end of arduino reboot
  {
    rebootPhase--;
    Serial.print("reboot phase:");
    Serial.println(rebootPhase);
    EchoServoAlign(servoAlignedPosition, false);                              // adjust servo motor to center position
  }
  if (rebootPhase == 5 && millis() > rebootDuration + 1000) // end of arduino reboot
  {
    PingFrontBack();
    rebootPhase--;
    Serial.print("reboot phase:");
    Serial.println(rebootPhase);

  }

  if (rebootPhase == 4 && analogRead(power6Value) > map(power6LowLimit, 0, 1873, 0, 1023)) // motors power on
  {
    rebootPhase--;
    Serial.print("reboot phase:");
    Serial.println(rebootPhase);
    digitalWrite(RobotOutputRobotRequestPin, HIGH);
    delay(500);
    pinMode(RobotOutputRobotRequestPin, INPUT);     // input mode till the subsystem is ready
  }
  if (rebootPhase == 4 && millis() > rebootESPDuration)
  {
    rebootPhase--;
    bitWrite(rebootDiag, 2, 1);
    Serial.print("timeout>reboot phase:");
    Serial.println(rebootPhase);
  }
  if ((rebootPhase == 3) && millis() > rebootBNODuration) // end of arduino reboot
  {
    int pulseCount = 0;
    if (digitalRead(RobotOutputRobotRequestPin) == 1 && digitalRead(RobotInputReadyPin) == 0)
    {
      pulseCount++;
      delay(1000);
      Serial.println("move a little to calibrate compass");
      bRightClockwise = bLeftClockwise;
      //        bitWrite(currentMove, toDoClockwise, false);
      //        bitWrite(saveCurrentMove, toDoClockwise, false);
      pulseMotors(2);
      bLeftClockwise = !bLeftClockwise;
      //   delay(1500);
    }
    if ((rebootPhase == 3) && (millis() > rebootBNOTimeout || bitRead(BNOSysStat, 0))) // BNo timeout or BNO not iddle
    {
      rebootPhase--;
      bitWrite(rebootDiag, 1, 1);
      Serial.print("timoeout>reboot phase:");
      Serial.println(rebootPhase);
    }
    if (pulseCount % 2 == 1)     // to balance clockwise and anitclockwise move
    {
      delay(1000);
      Serial.println("...");
      bRightClockwise = bLeftClockwise;
      pulseMotors(2);
    }
  }
  if (rebootPhase == 3 && digitalRead(RobotInputReadyPin) == 1 && millis() > rebootDuration + 7000) // end of arduino reboot
  {
    //   northOrientation = NorthOrientation();            // added 24/10/2016
    PingFrontBack();
    rebootPhase--;
    pinMode(RobotOutputRobotRequestPin, OUTPUT);     // subsystel is ready set output mode as running mode
    digitalWrite(RobotOutputRobotRequestPin, LOW);
    Serial.print("reboot phase:");
    Serial.println(rebootPhase);
  }
  if (rebootPhase == 2 && gyroCalibrationOk && millis() > rebootDuration + 7000) // end of arduino reboot
  {
    diagRobot = rebootDiag;
    waitFlag = 0x00;
    if (diagRobot == 0x00)
    {
      appStat = 0x00;
    }
    bitWrite(diagConnection, diagConnectionI2C, 0);
    PingFrontBack();
    Horn(true, 250);
    rebootPhase = 0;
    iddleTimer = millis();
    Serial.print("reboot diag:");
    Serial.println(rebootDiag, HEX);
  }

  if (rebootPhase == 1 && millis() > rebootDuration) // end of arduino reboot
  {
    rebootPhase--;
    delay(1000);

    Serial.print("reboot phase:");
    Serial.println(rebootPhase);
    bitWrite(currentMove, toDoRotation, HIGH);
    bitWrite(currentMove, toDoBackward, HIGH);
    boolean IRsensorsOk = false;
    PowerOnIRSensors();
    while (!IRsensorsOk)
    {
      stateObstacle state = robotIR.CheckObstacle();
      if (state.obstacle)
      {
        Serial.println(state.sensorsOnMap, HEX);
        delay(10000);
        digitalWrite(IRPower1PIN, LOW);
        digitalWrite(IRPower2PIN, LOW);
        delay(1000);
        PowerOnIRSensors();
      }
      else
      {
        digitalWrite(IRPower1PIN, LOW);
        digitalWrite(IRPower2PIN, LOW);
        robotIR.StopSensor(0);
        robotIR.StopSensor(1);
        robotIR.StopSensor(2);
        robotIR.StopSensor(3);
        robotIR.StopSensor(4);
        robotIR.StopSensor(5);
        currentMove = 0x00;
        IRsensorsOk = true;
      }
    }
    //   SetBNOMode(MODE_IMUPLUS);
  }

}

/*
  void SetGyroBiasMicrosec(uint8_t registerValue)
  {
  uint8_t registers[3];
  uint8_t registersValues[3];
  registers[0] = 0x18;
  registersValues[0] = registerValue;
  SubsystemSetRegisters(1, registers, registersValues);
  }
  void GyroResetRegisters()
  {
  uint8_t registers[maxRegsNumberUpdate];
  uint8_t registersValues[maxRegsNumberUpdate];
  SubsystemSetRegisters(0xff, registers, registersValues);
  RequestForPolling();
  }
  void GyroSetPollingRegister(int cycleValue)
  {
  uint8_t reg[maxRegsNumberUpdate];
  reg[0] = robotPollingTimer_Reg1;
  reg[1] = robotPollingTimer_Reg2;
  uint8_t regVal[maxRegsNumberUpdate];

  regVal[0] =  (cycleValue / 256);
  regVal[1] =  (cycleValue);
  // robotPollingTimer_Reg1 = (cycleValue / 256);
  SubsystemSetRegisters(2, reg, regVal);
  RequestForPolling();
  }
*/
void SubsystemReadRegisters(uint8_t number, uint8_t registers[maxRegsNumberRead])
{
  InitOutData();
  outData[1] = readRegisterRequest;
  outData[2] = number;                 // nb register to read
  outData[3] = registers[0];
  outData[4] = registers[1];
  outData[5] = registers[2];
  outData[6] = registers[3];
  outData[7] = registers[4];
  outData[8] = registers[5];
  RequestForPolling();
}
void SubsystemSetRegisters(uint8_t number, uint8_t registers[3], uint8_t registersValues[3])
{
  InitOutData();
  outData[1] = setRegisterRequest;
  outData[2] = number;                 // nb register to read
  outData[3] = registers[0];
  outData[4] = registersValues[0];
  outData[5] = registers[1];
  outData[6] = registersValues[1];
  outData[7] = registers[2];
  outData[8] = registersValues[2];
  RequestForPolling();
}
void SubsystemSetMoveRegisters(uint8_t number, uint8_t registers[3], uint8_t registersValues[3])
{
  InitOutData();
  outData[1] = setMoveRegisters;
  outData[2] = number;                 // nb register to read
  outData[3] = registers[0];
  outData[4] = registersValues[0];
  outData[5] = registers[1];
  outData[6] = registersValues[1];
  outData[7] = registers[2];
  outData[8] = registersValues[2];
  RequestForPolling();
}
void GyroStartMonitor(boolean forward)
{
  InitOutData();
  outData[1] = startMonitorGyro;
  outData[2] = uint8_t(forward);
  RequestForPolling();
}
void GyroInitLocation()
{
  InitOutData();
  outData[1] = initLocation;
  RequestForPolling();
}
void GyroStartInitMonitor(boolean forward)
{
#if defined(debugGyroscopeL2On)
  Serial.println("startinit");
#endif
  InitOutData();
  outData[1] = startInitMonitorGyro;
  outData[2] = uint8_t(forward);
  RequestForPolling();
}
void GyroStopMonitor()
{
  InitOutData();
  outData[1] = stopMonitorGyro;
  RequestForPolling();
}
void GyroStopInitMonitor()
{
  InitOutData();
  outData[1] = stopInitMonitorGyro;
  RequestForPolling();
}
void StartMagneto()
{
  InitOutData();
  outData[1] = startMonitorMagneto;
  RequestForPolling();
}
void StopMagneto()
{
  InitOutData();
  outData[1] = stopMonitorMagneto;
  RequestForPolling();
}
void CalibrateGyro()
{
  gyroCalibrationOk = false;
  InitOutData();
  outData[1] = calibrateGyro;
  RequestForPolling();
}
void GyroGetHeadingRegisters()
{
  uint8_t reqRegisters[3] = {relativeHeadingResponse[0], relativeHeadingResponse[1], relativeHeadingResponse[2]};
  SubsystemReadRegisters(0x03, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetAbsoluteHeading()
{
  uint8_t reqRegisters[3] = {absoluteHeadingResponse[0], absoluteHeadingResponse[1], absoluteHeadingResponse[2]};
  SubsystemReadRegisters(0x03, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetNorthOrientation()
{
  uint8_t reqRegisters[2] = {compassResponse[0], compassResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetBeforeNorthOrientation()
{
  uint8_t reqRegisters[2] = {beforeNOResponse[0], beforeNOResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetAfterNorthOrientation()
{
  uint8_t reqRegisters[2] = {afterNOResponse[0], afterNOResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetBeforeAfterNorthOrientation()
{
  uint8_t reqRegisters[4] = {beforeNOResponse[0], beforeNOResponse[1], afterNOResponse[0], afterNOResponse[1]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetBNO055Status()
{
  uint8_t reqRegisters[4] = {BNO055StatusResponse[0], BNO055StatusResponse[1], BNO055StatusResponse[2], BNO055StatusResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetBNOHeadingLocation()
{
  // Serial.println(getBNOLocation, HEX);
  uint8_t reqRegisters[2] = {BNO055LocationHeading[0], BNO055LocationHeading[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
  if (getBNOLocation != 0x00)
  {
    getBNOLocation--;
  }
  RequestForPolling();
}
void GetBNOLeftLocation()
{
  uint8_t reqRegisters[4] = {BNO055LeftLocationResponse[0], BNO055LeftLocationResponse[1], BNO055LeftLocationResponse[2], BNO055LeftLocationResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
  if (getBNOLocation != 0x00)
  {
    getBNOLocation--;
  }
  RequestForPolling();
}
void GetBNORightLocation()
{
  uint8_t reqRegisters[4] = {BNO055RightLocationResponse[0], BNO055RightLocationResponse[1], BNO055RightLocationResponse[2], BNO055RightLocationResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
  if (getBNOLocation != 0x00)
  {
    getBNOLocation--;
  }
  RequestForPolling();
}
void RequestForPolling()
{
  digitalWrite(RobotOutputRobotRequestPin, HIGH);
  OutputRobotRequestPinTimer = millis();
  pendingPollingResp = 0x02;
}
void InitOutData()
{
  outData[0] = slaveAddress;
  outData[1] =  idleRequest;
  for (int i = 2; i < pollResponseLenght; i++)
  {
    outData[i] = 0x00;
  }
}
void ResetGyroscopeHeadings()
{
  for (int i = 0; i < maxGyroscopeHeadings; i++)
  {
    gyroscopeHeading[i] = 0;
  }
  gyroUpToDate = 0x00;
  gyroscopeHeadingIdx = 0x00;
}
void SetGyroSelectedRange(uint8_t value)
{
  // InitOutData();
  //  outData[1] = setGyroSelectedRange;
  // outData[2] = value;
  // RequestForPolling();
}
void SetGyroODR(uint8_t value)
{
  //  InitOutData();
  //  outData[1] = setGyroODR;
  // outData[2] = value;
  // RequestForPolling();
}
void GetSubsystemRegister(uint8_t number, uint8_t value[5])
{
  SubsystemReadRegisters(number, value);
}

void SetBNOMode(uint8_t value)
{

  if (millis() - lastSetModeTime > 1000)
  {
#if defined(debugGyroscopeL2On)
    Serial.println("setbno");
#endif
    InitOutData();
    outData[1] = setBNO055Mode;
    outData[2] = value;
    RequestForPolling();
    lastSetModeTime = millis();
    BNOUpToDateFlag = 0;
  }
  if (millis() - lastSetModeTime > 300)
  {
    GetBNO055Status();
    BNOUpToDateFlag = 0;
  }
  // lastSetModeTime = millis();
}
void InitBNOLocation()
{
  uint16_t uInitX = (int)round(posRotationGyroCenterX * leftWheelEncoderHoles / (fLeftWheelDiameter * PI));        // center position expressed in term of holes
  uint16_t uInitY = (int)round(posRotationGyroCenterY * rightWheelEncoderHoles / (fLeftWheelDiameter * PI));
  uint16_t uAlpha = (int)(round(alpha)) % 360;
#if defined(debugGyroscopeOn)
  Serial.print("InitBNOLocation:");
  Serial.println(stepBNOInitLocation, HEX);
#endif
  switch (stepBNOInitLocation)
  {
    case 1:
      {
        uint8_t regLoc[3] = {initPosY_Reg2, initHeading_reg1, initHeading_reg2};
        uint8_t regLocValue[3] = {(uint8_t)uInitY, (uint8_t)(uAlpha  >> 8 ), (uint8_t)uAlpha};
        SubsystemSetMoveRegisters(3, regLoc, regLocValue);
        stepBNOInitLocation++;
        break;
      }
    case 2:
      {
        uint8_t regLoc[3] = {initPosX_Reg1, initPosX_Reg2, initPosY_Reg1};
        uint8_t regLocValue[3] = {((uint8_t)((uInitX & 0x7fff) >> 8) | ((uInitX & 0x8000) >> 8)), (uint8_t)uInitX , ((uint8_t)((uInitY & 0x7fff) >> 8) | ((uInitY & 0x8000) >> 8))};
        SubsystemSetMoveRegisters(3, regLoc, regLocValue);
        stepBNOInitLocation++;
        break;
      }
    case 3:
      {
        GyroInitLocation();
        stepBNOInitLocation = 0x00;
        break;
      }
  }

}
void UpdateBNOMove()
{
  lastUpdateBNOMoveTime = millis();
  unsigned int currentLeftHoles = Wheels.GetCurrentHolesCount(leftWheelId);
  unsigned int currentRightHoles = Wheels.GetCurrentHolesCount(rightWheelId);

  if (currentLeftHoles > BNOprevSentLeftHoles || currentRightHoles > BNOprevSentRightHoles)
  {
#if defined(debugGyroscopeOn)
    Serial.print ("update holes left:");
    Serial.print(currentLeftHoles);
    Serial.print(" right:");
    Serial.println(currentRightHoles);
#endif
    uint8_t reg[3] = {requestCompute_Reg, leftDistance_Reg, rightDistance_Reg};
    uint8_t regValue[3] = {0x01, uint8_t(abs(currentLeftHoles - BNOprevSentLeftHoles)), uint8_t(abs(currentRightHoles - BNOprevSentRightHoles))};
    SubsystemSetMoveRegisters(3,  reg, regValue);
    BNOprevSentLeftHoles = currentLeftHoles;
    BNOprevSentRightHoles = currentRightHoles;
  }
}



void requestEvent() {
#if defined(debugGyroscopeL2On)
  Serial.print("request event:");
  Serial.print(inputData[0]);
  Serial.print("-");
  Serial.println(inputData[1]);
#endif
  //  if (inputData[0] == slaveAddress && inputData[1] == 0x01 && inputData[2] <= pollResponseLenght)
  if (inputData[0] == slaveAddress  && inputData[2] <= pollResponseLenght)
  {
    Wire.write(outData, pollResponseLenght);
    delay(1);
    InitOutData();
  }
}

void receiveEvent(int howMany) {

  //  Serial.println(howMany);
  int receivedCount = 0;
#if defined(debugGyroscopeL2On)
  Serial.print("receive event: ");
#endif
  while (Wire.available()) { // loop through all but the last
    inputData[receivedCount] = Wire.read(); // receive byte as a character
#if defined(debugGyroscopeL2On)
    Serial.print(inputData[receivedCount], HEX);        // print the character
    Serial.print("-");
#endif
    receivedCount++;
  }
  //   digitalWrite(OutputRobotRequestPin, LOW);
#if defined(debugGyroscopeL2On)
  Serial.println();
#endif
  /*
     decode received frame
  */
  uint8_t cmd = inputData[1];
  uint8_t  receivedNumber = inputData[2];
  uint8_t receivedRegister[15];
  digitalWrite(RobotOutputRobotRequestPin, LOW);

  switch (cmd)
  {
    case idleRequest:
      {
        monitSubsystemStatus = inputData[2];
#if defined(debugGyroscopeL2On)
        if (bitRead(inputData[2], monitGyroStatusBit) == 1)
        {
          Serial.print("Giroscope ");
        }
        if (bitRead(inputData[2], monitMagnetoStatusBit) == 1)
        {
          Serial.print("Magneto ");
        }
        if (bitRead(inputData[2], monitGyroStatusBit) == 1 || bitRead(inputData[2], monitMagnetoStatusBit) == 1)
        {
          Serial.println(("running"));
        }
#endif
        break;
      }
    case readRegisterResponse:
      {
        for (int i = 0; i < receivedNumber; i++)
        {
          receivedRegister[i] = inputData[2 * i + 3];
        }
        switch (receivedNumber)
        {
          case (2):                           // 2 registers received
            {
              switch (receivedRegister[0])
              {

                case (compasHeading_Reg1):
                  {
                    if (compasUpToDate == 0x01)
                    {
                      compasUpToDate = 0x02;
                    }
                    northOrientation = inputData[4] * 256 + inputData[6];
#if defined(debugGyroscopeOn)
                    Serial.print("Compass:");
                    Serial.println(northOrientation);
#endif
                    break;
                  }

                case (locationHeading_reg1):
                  {
                    if (getBNOLocation != 0x00)
                    {
                      getBNOLocation--;
                    }
                    BNOLocationHeading = inputData[4] * 256 + inputData[6];
#if defined(debugGyroscopeOn)
                    Serial.print("BNO locationHeading:");
                    Serial.println(BNOLocationHeading);
#endif
                    break;
                  }
                  break;
              }
              break;
            }
          case (3):                           // 3 registers received
            {
              switch (receivedRegister[0])
              {
                case (relativeHeading_Reg1):
                  {
                    boolean trameOk = true;
                    for (int i = 0; i < sizeof(relativeHeadingResponse); i++)
                    {
                      if (receivedRegister[i] != relativeHeadingResponse[i])
                      {
                        trameOk = false;
                      }
                      if (trameOk == false)
                      {
                        break;
                      }
                    }
                    if (trameOk)
                    {
                      int relativeHeading = inputData[6] * 256 + inputData[8];
                      if (inputData[4] == 0x01)
                      {
                        relativeHeading = -relativeHeading;
                      }
                      gyroscopeHeadingIdx = (gyroscopeHeadingIdx + 1) % maxGyroscopeHeadings;
                      //#if defined(IMU)
                      //                      if (BNOMode == MODE_NDOF)
                      //                     {
                      relativeHeading = (360 - relativeHeading) % 360;
                      //                     }

                      //#endif
                      gyroscopeHeading[gyroscopeHeadingIdx] = relativeHeading ;
                      if (gyroUpToDate == 0x01)
                      {
                        gyroUpToDate = 0x02;
                      }
#if defined(debugGyroscopeL2On)
                      Serial.print("heading:");
                      Serial.println(relativeHeading);
#endif
                    }
                    break;
                  }
                  break;
                case (absoluteHeading_Reg1):
                  {
                    absoluteHeading = (inputData[6] * 256 + inputData[8]) % 360;
#if defined(debugGyroscopeL2On)
                    Serial.print("absolute heading:");
                    Serial.println(absoluteHeading);
#endif
                    break;
                  }
              }

            }
          case (4):                           // 4 registers received
            {
              switch (receivedRegister[0])    // BNO status received
              {
                case (BNO055Mode_Reg):
                  {
                    BNOMode = inputData [4];
                    BNOCalStat = inputData [6];
                    BNOSysStat = inputData [8];
                    BNOSysError = inputData [10];
#if defined(debugGyroscopeL2On)
                    Serial.print("BNO Mode:");
                    Serial.print(BNOMode, HEX);
                    Serial.print(" calibration status:");
                    Serial.print(BNOCalStat, HEX);
                    Serial.print(" system status:");
                    Serial.print(BNOSysStat, HEX);
                    Serial.print(" system error:");
                    Serial.println(BNOSysError, HEX);
#endif
                  }
                  break;
                case (deltaLeftPosX_reg1):
                  {
                    BNOLeftPosX = (int)((uint16_t)inputData[4] << 8 | (uint16_t) inputData[6]);
                    BNOLeftPosX = (float)(BNOLeftPosX * PI * fLeftWheelDiameter / leftWheelEncoderHoles) + shiftEchoVsRotationCenter * cos(BNOLocationHeading * PI / 180);
                    BNOLeftPosY = (int)((uint16_t)inputData[8] << 8 | (uint16_t) inputData[10]);
                    BNOLeftPosY = (float)(BNOLeftPosY * PI * fLeftWheelDiameter / leftWheelEncoderHoles) + shiftEchoVsRotationCenter * sin(BNOLocationHeading * PI / 180);
                    if (getBNOLocation != 0x00)
                    {
                      getBNOLocation--;
                    }
#if defined(debugGyroscopeOn)
                    Serial.print("BNO Left position X:");
                    Serial.print(BNOLeftPosX );
                    Serial.print(" Y:");
                    Serial.println(BNOLeftPosY);
#endif
                    break;
                  }
                case (deltaRightPosX_reg1):
                  {
                    BNORightPosX = (int)((uint16_t)inputData[4] << 8 | (uint16_t) inputData[6]);
                    BNORightPosX = (float)(BNORightPosX  * PI * fRightWheelDiameter / rightWheelEncoderHoles) + shiftEchoVsRotationCenter * cos(BNOLocationHeading * PI / 180);
                    BNORightPosY  = (int)((uint16_t)inputData[8] << 8 | (uint16_t) inputData[10]);
                    BNORightPosY  = (float)(BNORightPosY  * PI * fRightWheelDiameter / rightWheelEncoderHoles) + shiftEchoVsRotationCenter * sin(BNOLocationHeading * PI / 180);
                    if (getBNOLocation != 0x00)
                    {
                      getBNOLocation--;
                    }
#if defined(debugGyroscopeOn)
                    Serial.print("BNO Right position X:");
                    Serial.print(BNORightPosX);
                    Serial.print(" Y:");
                    Serial.println(BNORightPosY);
#endif
                    break;
                  }
                  break;
              }
              break;
            }
          case (6):
            {
              boolean trameBeforeAfterNO = false;
              for (int i = 0; i < sizeof(beforeAfterNOResponse); i++)
              {
                if (receivedRegister[i] == beforeAfterNOResponse[i])
                {
                  trameBeforeAfterNO = true;
                }
              }
              if (trameBeforeAfterNO)
              {
                Serial.print("before NO:");
                int NO = inputData[6] * 256 + inputData[8];
                Serial.print(NO);
                Serial.print(" after NO:");
                NO = inputData[10] * 256 + inputData[12];
                Serial.println(NO);
              }
              break;
            }
        }
        break;
      }

    case (calibrateGyro):
      {
        gyroCalibrationOk = true;
#if defined(debugGyroscopeOn)
        Serial.println("gyro calibration ok");
#endif
        break;
      }
  }
  pendingPollingResp = 0x01;
}

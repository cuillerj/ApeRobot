/*
  BNOUpToDateFlag used to get regurarly alternativly BNO status or heading

  pendingPollingResp
    0x01 pending request
    0x00 response receive to a pooling request

  pendingDataResp
    true command expects data from subsystem
    false command does not expect data from subsystem

  BNORequestedState flag used inside BNOLoop to determine next command
  BNORequestSequence
    false single command
    true means consecutive commands accroding to BNOLoop switch

  stepBNOInitLocation flag used to determine initialsation location step

*/

void RequestForPolling(boolean expectedResponse)
{
#if defined(debugGyroscopeOn)
  Serial.print("RequestForPolling-");
  Serial.print(pendingPollingResp, HEX);
  Serial.print(" set to:");
  Serial.print(expectedResponse, HEX);
  Serial.print(" seq:");
  Serial.println(BNORequestSequence);
#endif
  pendingDataResp = expectedResponse;
  digitalWrite(RobotOutputRobotRequestPin, HIGH);
  OutputRobotRequestPinTimer = millis();
  pendingPollingResp = 0x01;
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



void InitBNOLocation()
{
  uint16_t uInitX = (int)round((posX + posRotationGyroCenterX) * leftWheelEncoderHoles / (fLeftWheelDiameter * PI));      // center position expressed in term of holes
  uint16_t uInitY = (int)round((posY + posRotationGyroCenterY) * rightWheelEncoderHoles / (fRightWheelDiameter * PI));
  uint16_t uAlpha = (int)(round(alpha)) % 360;
#if defined(debugGyroscopeOn)
  Serial.print("InitBNOLocation:");
  Serial.println(stepBNOInitLocation, HEX);
  Serial.print("X:");
  Serial.print(uInitX);
  Serial.print(" Y:");
  Serial.print(uInitY);
  Serial.print(" H:");
  Serial.println(uAlpha);
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
        stepBNOInitLocation++;
        GyroInitLocation();
        break;
      }
    case 4:
      {
        BNORequestedState = BNOGetHeadingState;
        stepBNOInitLocation = 0x00;
        break;
      }
  }
}
void UpdateBNOMove()
{
  if (wheelIdInterruption == 5) { // not significative in pulse mode
    return;
  }
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
  digitalWrite(RobotOutputRobotRequestPin, LOW);
  pendingPollingResp = 0x00;
  Wire.write(outData, pollResponseLenght);
#if defined(debugGyroscopeOn)
  Serial.print("request event poll:0x");
  Serial.print(pendingPollingResp, HEX);
  Serial.print(" state:0x");
  Serial.print(BNORequestedState, HEX);
  Serial.print(" indata:");
  Serial.print(inputData[0]);
  Serial.print("-");
  Serial.print(inputData[1]);
  Serial.print("-");
  Serial.println( inputData[2]);
#endif
  /*
    for (int i = 0; i < pollResponseLenght; i++) {
    Serial.print(outData[i], HEX);
    Serial.print("-");
    }
    Serial.println();
  */
  return;
}

void receiveData(int howMany) {
  eventHowMany = howMany;
  pendingDataResp = false;
  //  Serial.println(howMany);
  //  eventReceived = false;
  int receivedCount = 0;
  bitWrite(diagConnection, 2, 0); //
#if defined(debugGyroscopeOn)
  Serial.print("receive event:poll 0x");
  Serial.print(pendingPollingResp, HEX);
  Serial.print(" BNORequestedState:0x");
  Serial.print(BNORequestedState, HEX);
  Serial.print(" ");
#endif
  pendingPollingResp = 0x00;
  while (Wire.available()) { // loop through all but the last
    inputData[receivedCount] = Wire.read(); // receive byte as a character
#if defined(debugGyroscopeL2On)
    Serial.print(inputData[receivedCount], HEX);        // print the character
    Serial.print("-");
#endif
    receivedCount++;
  }
#if defined(debugGyroscopeL2On)
  Serial.println();
#endif
  /*
    decode received frame
  */
  volatile uint8_t cmd = inputData[1];
  volatile uint8_t  receivedNumber = inputData[2];
  volatile uint8_t receivedRegister[15];

  switch (cmd)
  {
    case idleRequest:
      {
        monitSubsystemStatus = inputData[2];
        BNORequestedState = BNOGotStatus;
        if (bitRead(monitSubsystemStatus, monitGyroStatusBit))
        {
#if defined(debugGyroscopeOn)
          Serial.print("Giroscope ");
#endif
        }
        if (bitRead(monitSubsystemStatus, monitMagnetoStatusBit))
        {
#if defined(debugGyroscopeOn)
          Serial.print("Magneto ");
#endif
        }
        if (bitRead(monitSubsystemStatus, monitGyroStatusBit) == 1 || bitRead(monitSubsystemStatus, monitMagnetoStatusBit) == 1)
        {
#if defined(debugGyroscopeOn)
          Serial.println(("running"));
#endif
        }
        if (monitSubsystemStatus == 0x00) {
#if defined(debugGyroscopeOn)
          Serial.println(("not running"));
#endif
        }
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
          case (2):                           // 2 registers data received
            {
              switch (receivedRegister[0])    // wich kind of data is it ?
              {

                case (compasHeading_Reg1):    // compass heading stored in 2 bytes
                  {
                    if (compasUpToDate == 0x01)
                    {
                      compasUpToDate = 0x02;
                    }
                    northOrientation = inputData[4] * 256 + inputData[6];
                    BNORequestedState = BNOGotCompasHeadingState;
#if defined(debugGyroscopeOn)
                    Serial.print("Compass:");
                    Serial.println(northOrientation);
#endif
                    break;
                  }

                case (locationHeading_reg1):    //  location heading stored in 2 bytes
                  {
                    BNORequestedState = BNOGotHeadingState;
                    BNOLocationHeading = inputData[4] * 256 + inputData[6];
#if defined(debugGyroscopeOn)
                    Serial.print("BNOlocationHeading:");
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
                      BNORequestedState = BNOGotRelativeHeadingState;
#if defined(debugGyroscopeL2On)
                      Serial.print("relativeHeading:");
                      Serial.println(relativeHeading);
#endif
                    }
                    break;
                  }
                  break;
                case (absoluteHeading_Reg1):
                  {
                    absoluteHeading = (inputData[6] * 256 + inputData[8]) % 360;
                    BNORequestedState = BNOGotAbsoluteHeadingState;
#if defined(debugGyroscopeL2On)
                    Serial.print("absoluteHeading:");
                    Serial.println(absoluteHeading);
#endif
                    break;
                  }

              }
              break;
            }
          case (4):                           // 4 registers received
            {
              switch (receivedRegister[0])    // BNO status received
              {
                case (BNO055Mode_Reg):
                  {
                    BNORequestedState = BNOGotMode ;
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
                    break;
                  }

                case (deltaLeftPosX_reg1):
                  {
                    BNORequestedState = BNOGotLeftState;
                    BNOLeftPosX = (int)((uint16_t)inputData[4] << 8 | (uint16_t) inputData[6]);
                    BNOLeftPosX = (float)(BNOLeftPosX * PI * fLeftWheelDiameter / leftWheelEncoderHoles) + shiftEchoVsRotationCenter * cos(BNOLocationHeading * PI / 180);
                    BNOLeftPosY = (int)((uint16_t)inputData[8] << 8 | (uint16_t) inputData[10]);
                    BNOLeftPosY = (float)(BNOLeftPosY * PI * fLeftWheelDiameter / leftWheelEncoderHoles) + shiftEchoVsRotationCenter * sin(BNOLocationHeading * PI / 180);
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
                    BNORequestedState = BNOGotRightState;
                    BNORightPosX = (int)((uint16_t)inputData[4] << 8 | (uint16_t) inputData[6]);
                    BNORightPosX = (float)(BNORightPosX  * PI * fRightWheelDiameter / rightWheelEncoderHoles) + shiftEchoVsRotationCenter * cos(BNOLocationHeading * PI / 180);
                    BNORightPosY  = (int)((uint16_t)inputData[8] << 8 | (uint16_t) inputData[10]);
                    BNORightPosY  = (float)(BNORightPosY  * PI * fRightWheelDiameter / rightWheelEncoderHoles) + shiftEchoVsRotationCenter * sin(BNOLocationHeading * PI / 180);
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
}

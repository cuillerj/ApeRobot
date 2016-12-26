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
void GyroStartMonitor()
{
  InitOutData();
  outData[1] = startMonitorGyro;
  RequestForPolling();

}
void GyroStartInitMonitor()
{
  InitOutData();
  outData[1] = startInitMonitorGyro;
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
  RequestForPolling();
}
void StartMagneto()
{
  InitOutData();
  outData[1] = startMonitorMagneto;
  RequestForPolling();
  RequestForPolling();
}
void StopMagneto()
{
  InitOutData();
  outData[1] = stopMonitorMagneto;
  RequestForPolling();
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
  uint8_t reqRegisters[3] = {headingResponse[0], headingResponse[1], headingResponse[2]};
  SubsystemReadRegisters(0x03, reqRegisters);          // read z registers
  RequestForPolling();
}
void GetNorthOrientation()
{
  uint8_t reqRegisters[2] = {NOResponse[0], NOResponse[1]};
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
void RequestForPolling()
{
  digitalWrite(RobotOutputRobotRequestPin, HIGH);
  OutputRobotRequestPinTimer = millis();
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
  gyroUpToDate=0x00;
  gyroscopeHeadingIdx = 0x00;
}
void SetGyroSelectedRange(uint8_t value)
{
  InitOutData();
  outData[1] = setGyroSelectedRange;
  outData[2] = value;
  RequestForPolling();
}
void SetGyroODR(uint8_t value)
{
  InitOutData();
  outData[1] = setGyroODR;
  outData[2] = value;
  RequestForPolling();
}
void GetSubsystemRegister(uint8_t number, uint8_t value[5])
{
  SubsystemReadRegisters(number, value);
}

void requestEvent() {
#if defined(debugGyroscopeOn)
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
          case (3):
            {
              boolean trameOk = true;
              for (int i = 0; i < sizeof(headingResponse); i++)
              {
                if (receivedRegister[i] != headingResponse[i])
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
                gyroscopeHeading[gyroscopeHeadingIdx] = relativeHeading*L3GPositiveClockWise;
                if (gyroUpToDate==0x01)
                {
                  gyroUpToDate = 0x02;                
                }
#if defined(debugGyroscopeOn)
                Serial.print("heading:");
                Serial.println(relativeHeading);
#endif
              }
              break;
            }
          case (2):
            {
              boolean trameNO = false;
              boolean trameBeforeNO = false;
              boolean trameAfterNO = false;
              for (int i = 0; i < sizeof(NOResponse); i++)
              {
                if (receivedRegister[i] == NOResponse[i])
                {
                  trameNO = true;
                }
              }
              for (int i = 0; i < sizeof(beforeNOResponse); i++)
              {
                if (receivedRegister[i] == beforeNOResponse[i])
                {
                  trameBeforeNO = true;
                }
              }
              for (int i = 0; i < sizeof(afterNOResponse); i++)
              {
                if (receivedRegister[i] == afterNOResponse[i])
                {
                  trameAfterNO = true;
                }
              }
              if (trameNO)
              {
                Serial.print("NO:");
              }
              if (trameBeforeNO)
              {
                Serial.print("before NO:");
              }
              if (trameAfterNO)
              {
                Serial.print("after NO:");
              }
              int NO = inputData[4] * 256 + inputData[6];
              Serial.println(NO);
              break;

            }
          case (4):
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
                int NO = inputData[4] * 256 + inputData[6];
                Serial.print(NO);
                Serial.print(" after NO:");
                NO = inputData[8] * 256 + inputData[10];
                Serial.println(NO);
              }
              break;

            }
          case (5):
            {
              uint8_t receivedValue[5];
              for (int i = 0; i < 5; i++)
              {
                receivedValue[i] = inputData[2 * i + 4];
              }
              SendUDPSubsystemRegister(receivedRegister, receivedValue);
              break;
            }
          default:
            {
              break;
            }
        }
      default:
        {
          break;
        }
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

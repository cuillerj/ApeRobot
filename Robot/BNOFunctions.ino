/*
  BNOUpToDateFlag used to get regurarly alternativly BNO status or heading

  pendingPollingResp
  0x02 pending request
  0x01 response receive to a pooling request
  0x00 response has been taken into account

  BNORequestedState

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
  RequestForPolling(true);
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
  RequestForPolling(false);
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
  RequestForPolling(false);
}
void GyroStartMonitor(boolean forward)
{
  InitOutData();
  outData[1] = startMonitorGyro;
  outData[2] = uint8_t(forward);
  RequestForPolling(false);
}
void GyroInitLocation()
{
  InitOutData();
  outData[1] = initLocation;
  RequestForPolling(false);
}
void GyroStartInitMonitor(boolean forward)
{
#if defined(debugGyroscopeL2On)
  Serial.println("GyroStartInitMonitor");
#endif
  InitOutData();
  outData[1] = startInitMonitorGyro;
  outData[2] = uint8_t(forward);
  RequestForPolling(false);
  BNORequestSequence = false;
}
void GyroStopMonitor()
{
  InitOutData();
  outData[1] = stopMonitorGyro;
  RequestForPolling(false);
  BNORequestSequence = true;
}
void GyroStopInitMonitor()
{
  InitOutData();
  outData[1] = stopInitMonitorGyro;
  RequestForPolling(false);
}
void StartMagneto()
{
  InitOutData();
  outData[1] = startMonitorMagneto;
  RequestForPolling(false);
  BNORequestSequence = false;
}
void StopMagneto()
{
  InitOutData();
  outData[1] = stopMonitorMagneto;
  RequestForPolling(false);
  BNORequestSequence = true;
}
void CalibrateGyro()
{
  gyroCalibrationOk = false;
  InitOutData();
  outData[1] = calibrateGyro;
  RequestForPolling(false);
}
void GyroGetHeadingRegisters()
{
  uint8_t reqRegisters[3] = {relativeHeadingResponse[0], relativeHeadingResponse[1], relativeHeadingResponse[2]};
  SubsystemReadRegisters(0x03, reqRegisters);          // read z registers
}
void GetAbsoluteHeading()
{
  uint8_t reqRegisters[3] = {absoluteHeadingResponse[0], absoluteHeadingResponse[1], absoluteHeadingResponse[2]};
  SubsystemReadRegisters(0x03, reqRegisters);          // read z registers
}
void GetNorthOrientation()
{
  uint8_t reqRegisters[2] = {compassResponse[0], compassResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
}
void GetBeforeNorthOrientation()
{
  uint8_t reqRegisters[2] = {beforeNOResponse[0], beforeNOResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
}
void GetAfterNorthOrientation()
{
  uint8_t reqRegisters[2] = {afterNOResponse[0], afterNOResponse[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers
}
void GetBeforeAfterNorthOrientation()
{
  uint8_t reqRegisters[4] = {beforeNOResponse[0], beforeNOResponse[1], afterNOResponse[0], afterNOResponse[1]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
}
void GetBNO055Status()
{
  uint8_t reqRegisters[4] = {BNO055StatusResponse[0], BNO055StatusResponse[1], BNO055StatusResponse[2], BNO055StatusResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers
    #if defined(debugGyroscopeOn)
  Serial.println("GetBNO055Status");
  #endif
}
void GetBNOHeadingLocation()
{
  // Serial.println(BNORequestedState, HEX);
  uint8_t reqRegisters[2] = {BNO055LocationHeading[0], BNO055LocationHeading[1]};
  SubsystemReadRegisters(0x02, reqRegisters);          // read z registers

  if (BNORequestedState != BNOIddleState)
  {
    BNORequestedState--;
  }

}
void GetBNOLeftLocation()
{
  uint8_t reqRegisters[4] = {BNO055LeftLocationResponse[0], BNO055LeftLocationResponse[1], BNO055LeftLocationResponse[2], BNO055LeftLocationResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers

  if (BNORequestedState != BNOIddleState)
  {
    BNORequestedState--;
  }

}
void GetBNORightLocation()
{
  uint8_t reqRegisters[4] = {BNO055RightLocationResponse[0], BNO055RightLocationResponse[1], BNO055RightLocationResponse[2], BNO055RightLocationResponse[3]};
  SubsystemReadRegisters(0x04, reqRegisters);          // read z registers

  if (BNORequestedState != BNOIddleState)
  {
    BNORequestedState--;
  }

}
void GetSubsystemRegister(uint8_t number, uint8_t value[5])
{
  SubsystemReadRegisters(number, value);
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
void SetBNOMode(uint8_t valu)
{
 // if (millis() - lastSetModeTime > 1000)
 // {
    Serial.print("setbno:");
    Serial.println(valu, HEX);
    InitOutData();
    outData[1] = setBNO055Mode;
    outData[2] = valu;
    RequestForPolling(false);
    lastSetModeTime = millis();
    //  BNOUpToDateFlag = 0;
 // }


}

void RequestSystemStatus()
{
  #if defined(debugGyroscopeOn)
  Serial.println("RequestSystemStatus");
  #endif
  InitOutData();
  outData[1] = requestSystemStatus;
  RequestForPolling(true);
}

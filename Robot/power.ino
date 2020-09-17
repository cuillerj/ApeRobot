void PowerCheck()
{
  /*
      keep the lowest voltage values for each power mesurments during the cycle
      cycle when is initialyse when sending data to the server
  */
  timePowerCheck = millis();                       // reset timer
  //  Serial.print("power1: ");
  float tempVoltage = 0;
  unsigned int power1 = analogRead(power1Value); // read power1 analog value and map to fit real voltage
  power1 = analogRead(power1Value);              // read twice to get a better value
  tempVoltage = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (tempVoltage < power1Mesurt || newPowerCycle)              // keep the lowest value a a cycle
  {
    power1Mesurt = tempVoltage;
  }
  //    power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power1Mesurt < power1LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 0, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 0, false);                                       // set diag bit power 1 pk
  }
  unsigned int power2 = analogRead(power2Value); // read power1 analog value and map to fit real voltage
  power2 = analogRead(power2Value);               // read twice to get a better value
  tempVoltage = (power2 * float (1005) / 1023);  // map to fit real voltage
  if (tempVoltage < power2Mesurt || newPowerCycle)              // keep the lowest value a a cycle
  {
    power2Mesurt = tempVoltage;
  }

  if (power2Mesurt < power2LowLimit)                // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 1, true);                  // set diag bit power 2 ok
    //  myservo.detach();
  }
  else
  {
    bitWrite(diagPower, 1, false);                      // set diag bit power 2 ok
  }
  unsigned int power3 = analogRead(power3Value); // read power1 analog value and map to fit real voltage
  power3 = analogRead(power3Value);              // read twice to get a better value
  //  tempVoltage = (power3 * float (1005) / 1023);  // map to fit real voltage
  tempVoltage = map(power3, 0, 1023, 0, 2000);
  if (tempVoltage < power3Mesurt || newPowerCycle)               // keep the lowest value a a cycle
  {
    power3Mesurt = tempVoltage;
  }
  //    power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power3Mesurt < power3LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 2, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 2, false);                                       // set diag bit power 1 pk
  }
  unsigned int power4 = analogRead(power4Value); // read power1 analog value and map to fit real voltage
  power4 = analogRead(power4Value);              // read twice to get a better value
  tempVoltage = map(power4, 0, 1023, 0, 1010);
  if (tempVoltage < power4Mesurt || newPowerCycle)               // keep the lowest value a a cycle
  {
    power4Mesurt = tempVoltage;
  }
  //    power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power4Mesurt < power4LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 3, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 3, false);                                       // set diag bit power 1 pk
  }
  unsigned int power5 = analogRead(power5Value); // read power1 analog value and map to fit real voltage
  power5 = analogRead(power5Value);              // read twice to get a better value
  //tempVoltage = (power5 * float (1005) / 1023);  // map to fit real voltage
  tempVoltage = map(power5, 0, 1023, 0, 2050);
  if (tempVoltage < power5Mesurt || newPowerCycle)               // keep the lowest value a a cycle
  {
    power5Mesurt = tempVoltage;
  }
  //    power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power5Mesurt < power5LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 4, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 4, false);                                       // set diag bit power 1 pk
  }
  unsigned int power6 = analogRead(power6Value); // read power1 analog value and map to fit real voltage
  power6 = analogRead(power6Value);              // read twice to get a better value
  //  tempVoltage = (power6 * float (1000) / 545);  // map to fit real voltage
  tempVoltage = map(power6, 0, 1023, 0, 1873);
  if (tempVoltage < power6Mesurt || newPowerCycle)               // keep the lowest value a a cycle
  {
    power6Mesurt = tempVoltage;
  }
  //    power1Mesurt = (power1 * float (1005) / 1023);  // map to fit real voltage
  if (power6Mesurt < power6LowLimit)                                    // check power voltage is over minimum threshold
  {
    bitWrite(diagPower, 5, true);                                       // set diag bit power 1 ok
  }
  else
  {
    bitWrite(diagPower, 5, false);                                       // set diag bit power 1 pk
  }


#if defined(debugPowerOn)
  Serial.print(power1);
  Serial.print(" ");
  Serial.print(power1Mesurt); // calibre avec 2+1 resitances 1Mg ohm 9v
  Serial.println("cV 1");
  Serial.print(power2); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print(" ");
  Serial.print(power2Mesurt);
  Serial.println("cV 2");
  Serial.print(power3);
  Serial.print(" ");
  Serial.print(power3Mesurt); // calibre avec 2+1 resitances 1Mg ohm 9v
  Serial.println("cV 3");
  Serial.print(power4); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print(" ");
  Serial.print(power4Mesurt);
  Serial.println("cV 4");
  Serial.print(power5);
  Serial.print(" ");
  Serial.print(power5Mesurt); // calibre avec 2+1 resitances 1Mg ohm 9v
  Serial.println("cV 5");
  Serial.print(power6); // calibre avec 1+1 resitances 1Mg ohm 5v
  Serial.print(" ");
  Serial.print(power6Mesurt);
  Serial.println("cV 6");
#endif
}

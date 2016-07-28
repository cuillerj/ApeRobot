#define powerSwitchPIN 48
#define encoderInput A8
void setup() {
  pinMode(powerSwitchPIN, OUTPUT);
  Serial.begin(38400);
  digitalWrite(powerSwitchPIN,1);
}
void loop() {
   Serial.print(float(map(analogRead(encoderInput),0,1023,0,500))/100);
  Serial.println(" volt");
  delay(1000);
}

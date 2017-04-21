#define powerSwitchPIN 48
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A7  // analog input right encoder
void setup() {
  pinMode(powerSwitchPIN, OUTPUT);
  Serial.begin(38400);
  digitalWrite(powerSwitchPIN, 1);
}
void loop() {
  Serial.print("left:");
  Serial.print(float(map(analogRead(leftAnalogEncoderInput), 0, 1023, 0, 500)) / 100);
  Serial.print(" right:"); 
  Serial.print(float(map(analogRead(rightAnalogEncoderInput), 0, 1023, 0, 500)) / 100);
  Serial.println(" volt");
  delay(1000);
}

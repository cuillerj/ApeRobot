float PosRotationCenterX() {
  float posRotationCenterX = - shiftEchoVsRotationCenter * cos(alpha * PI / 180) + posX ; // x position of the rotation center
  return posRotationCenterX;
}
float PosRotationCenterY() {
  float posRotationCenterY = - shiftEchoVsRotationCenter * sin(alpha * PI / 180) + posY ; // y position of the rotation center
  return posRotationCenterY;
}

float PosRotationGyroCenterX() {
  float posRotationGyroCenterX =  shiftEchoVsRotationCenter * cos(alpha * PI / 180) + posX; // x position of the rotation center
  return posRotationGyroCenterX;
}

float PosRotationGyroCenterY() {
  float posRotationGyroCenterY =  shiftEchoVsRotationCenter * sin(alpha * PI / 180) + posY; // y position of the rotation center
  return posRotationGyroCenterY;
}

void ComputeAngleAndPosition(float deltaD, float deltaLeft, float deltaRight, uint8_t param)
{
  float deltaAlphaRadian = asin(deltaD / (2 * iRobotWidth));
  float rayon =  deltaRight / (2 * sin(deltaAlphaRadian));
  float arcCenter = 2 * (rayon - iRobotWidth / 2) * sin(deltaAlphaRadian);
  float alphaRadian = alpha * PI / 180;
  deltaPosX = deltaPosX + arcCenter * cos(alphaRadian + deltaAlphaRadian);
  deltaPosY = deltaPosY + arcCenter * sin(alphaRadian + deltaAlphaRadian);
  alpha = (deltaAlphaRadian * 2 * 180 / PI + alpha);
#if defined(debugLocalizationOn)
  Serial.print("delta alpha: ");
  Serial.print(deltaAlphaRadian * 2 * 180 / PI);
  Serial.print(" ");
#endif
}
void ComputeAngleAndPositionVSGyro(int gyroRot)
{
  //  float deltaAlphaRadian = (gyroRot * PI) / 180;
  posX = PosRotationGyroCenterX() + shiftEchoVsRotationCenter * cos(gyroRot * PI / 180);    // to take into account the distance beetwen (X,Y) reference and rotation center
  posY = PosRotationGyroCenterY() + shiftEchoVsRotationCenter * sin(gyroRot * PI / 180);

#if defined(debugLocalizationOn)
  Serial.print(" alpha: ");
  Serial.print(gyroRot);
  Serial.print(" ");
#endif
}
int ComputeAngleDiff(int alpha,int beta) {
  int delta = alpha - beta;
  delta = (delta + 180) % 360 - 180;
  return delta;
}

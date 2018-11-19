// apres tests il faut 2 triggers distincts


#include <ApeRobotCommonDefine.h>
#include <NewPing.h>
#define SONAR_NUM 2      // Number of sensors.
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(trigFront, echoFront, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(trigBack, echoBack, MAX_DISTANCE),
};

unsigned long lecture_echo;
unsigned long front;
unsigned int minFront = MAX_DISTANCE + 1;
unsigned int maxFront;
unsigned int avgFront;
unsigned int cumFront;
unsigned long back;
unsigned int minBack = MAX_DISTANCE + 1;
unsigned int maxBack;
unsigned int avgBack;
unsigned int cumBack;
int i = 1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(trigFront, OUTPUT);
  digitalWrite(trigFront, HIGH);
  delay(5000);
  digitalWrite(trigFront, LOW);
  pinMode(trigBack, OUTPUT);
  digitalWrite(trigBack, HIGH);
  delay(5000);
  digitalWrite(trigBack, LOW);
  pinMode(echoFront, INPUT);
  pinMode(echoBack, INPUT);

}

void loop() {
  Serial.print(i);
  Serial.print(": Front ");
  //  front = PingFrontNew();
  front = sonar[0].ping_cm();
  cumFront = cumFront + front;
  avgFront = cumFront / i;
  Serial.print(front);
  if (front < minFront && front != 0)
  {
    minFront = front;
  }
  if (front > maxFront)
  {
    maxFront = front;
  }
  Serial.print(" min ");
  Serial.print(minFront);
  Serial.print(" max ");
  Serial.print(maxFront);
  Serial.print(" avg ");
  Serial.println(avgFront);
  delay(100);
  Serial.print(i);
  Serial.print(": Back ");
  // back = PingBackNew();
  back = sonar[1].ping_cm();
  Serial.print(back);
  cumBack = cumBack + back;
  avgBack = cumBack / i;
  if (back < minBack && back != 0)
  {
    minBack = back;
  }
  if (back > maxBack)
  {
    maxBack = back;
  }
  Serial.print(" min ");
  Serial.print(minBack);
  Serial.print(" max ");
  Serial.print(maxBack);
  Serial.print(" avg ");
  Serial.println(avgBack);
  delay(2000);
  i = i + 1;
}
// put your main code here, to run repeatedly:
int PingFront() {
  /*  for (int i=0;i>=sizeof(pulseValue);i++)
    {
    PosServo();
  */
  int cm;
  unsigned long time1;
  unsigned long time2;
  unsigned long deltaT;
  digitalWrite(trigFront, HIGH);

  delayMicroseconds(12); // 10 micro sec mini
  time1 = micros();
  digitalWrite(trigFront, LOW);
  lecture_echo = pulseIn(echoFront, HIGH);
  time2 = micros();
  deltaT = time2 - time1;
  //  Serial.print(lecture_echo);
  //  Serial.print(" deltaT Front:");
  //    Serial.println(deltaT);
  if (lecture_echo == 0)
  {
    cm = 450;
  }
  else
  {
    //    cm = lecture_echo / 58 * 1.23 - 3; // 1.25 coeff correction mesuree en reel -8 cm ecart VS roues
    //    cm = lecture_echo / 58  ; // 1.25 coeff correction mesuree en reel -8 cm ecart VS roues
    //    cm = deltaT / 58  ;
    cm = lecture_echo / 58  ;
  }
  /*
    Serial.print("Front t1:");
    Serial.print(time1);
    Serial.print(" t2:");
    Serial.print(time2);
    Serial.print("dT:");
    Serial.println(time2 - time1);
  */
  return (cm);

}
int PingBack() {
  int cm;
  unsigned long time1;
  unsigned long time2;
  /* for (int i=0;i>=sizeof(pulseValue);i++)
    {
  */
  //    PosServo();
  digitalWrite(trigBack, HIGH);
  time1 = micros();
  delayMicroseconds(12); // 10 micro sec mini
  digitalWrite(trigBack, LOW);
  // int lecture_echo=0;
  lecture_echo = pulseIn(echoBack, HIGH);
  //  Serial.println();
  // Serial.print("echo:");
  // Serial.print(lecture_echo);
  //  Serial.println();
  time2 = micros();
  //    Serial.print("deltaT Back:");
  // Serial.println(time2-time1);
  if (lecture_echo == 0)
  {
    cm = 450;
  }
  else
  {


    /*
      Serial.print("Back t1:");
      Serial.print(time1);
      Serial.print(" t2:");
      Serial.print(time2);
      Serial.print("dT:");
      Serial.println(time2 - time1);
    */
    //    cm = lecture_echo / 58 * 1.25 + 7; // 1.25 coeff correction mesuree en reel +8 ecrat VS roues
    cm = lecture_echo / 58;   // 1.25 coeff correction mesuree en reel +8 ecrat VS roues

  }
  return (cm);
}



#include <Servo.h>
#include <SoftwareSerial.h>

int ls1;
int ls2;

int t = 100;  //t has to be higher than value the sensor returns over a white surface and lower than the value the sensor returns over a black surface
// the value the QTI sensor returns when its hovering over a black or white surface is dependant on how high the QTI sensor is off the ground.
//If you notice the robot is not working  correctly check the values the QTI sensors are returning on the Serial Port.

void setup() {
  Serial.begin(115200);
}

void loop() {
  ls1 = RCTime(12);  // replace this with the pin you attatched the WHITE wire of the LEFT qti sensor too
  ls2 = RCTime(13);  // replace this with the pin you attatched the WHITE wire of the RIGHT qti sensor too

  Serial.println(ls2);
  Serial.println(ls1);

}
long RCTime(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT);
  digitalWrite(sensorIn, HIGH);
  delay(1);
  pinMode(sensorIn, INPUT);
  digitalWrite(sensorIn, LOW);
  while (digitalRead(sensorIn)) {
    duration++;
  }
  return duration;
}
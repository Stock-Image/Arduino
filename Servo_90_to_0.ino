#include <Servo.h>

int buzzer = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(buzzer,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly
  tone(buzzer,1000);
  digitalWrite(buzzer,HIGH);
  delay(1000);
  noTone(buzzer);
  delay(1000);
}

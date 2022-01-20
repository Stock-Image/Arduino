#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <math.h>

const uint8_t LED = 10;
const uint8_t buzzer = 12;
const uint8_t interruptpin = A2;

Adafruit_MPU6050 mpu;

long int t1,t2;
int init_ang = 0;
int angle = 0;
int x,y;
sensors_event_t a, g, temp;
long int check;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  
  pinMode(LED,OUTPUT);
  pinMode(buzzer,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptpin),Init,CHANGE);

  /* don't let the task end */
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  t1 = millis();
  mpu.getEvent(&a, &g, &temp);
  angle = RAD_TO_DEG * (atan2(-a.acceleration.y,-a.acceleration.x)*PI);
  Serial.println("angle");
  x = fabs(angle);
  y = fabs(init_ang);
  Serial.print("init_ang");
  Serial.println(init_ang);
  Serial.print("angle");
  Serial.println(angle);
  x = x - y;
  y = fabs(x);
  Serial.println(y);
  //tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  //noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
  if(y > 100)
  {
    tone(buzzer,1000);
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);
    t1 = millis();
  }
  noTone(buzzer);
  t2 = millis();
  check = t2-t1;
  Serial.print("Time for check");
  Serial.println(check);
}

/*interrupt to set a new init_angle*/
void Init(){
  //noInterrupts();
  Serial.println("Interrupt");
  init_ang = angle;
  //interrupts();
}

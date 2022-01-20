#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <math.h>

const uint8_t LED = 8;
const uint8_t buzzer = 12;
const uint8_t vib_dsc = 5;

const uint8_t interruptpin = A2;

Adafruit_MPU6050 mpu;

SemaphoreHandle_t xalarm;
SemaphoreHandle_t xangle;

int init_ang = 0;
int angle = 0;
int x,y,z,check;
sensors_event_t a, g, temp;
long int t1,t2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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
  
  pinMode(LED,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(vib_dsc,OUTPUT);
  pinMode(interruptpin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptpin),Init_ISR,CHANGE);
  
  
  xTaskCreate(get_val,  "angle", 128, NULL, 0, NULL );
  xTaskCreate(alarm,  "angle", 128, NULL, 0, NULL );

  
  xangle = xSemaphoreCreateBinary();
  xalarm = xSemaphoreCreateBinary();

  xSemaphoreGive(xangle);

  /* don't let the task end */
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("Gyro range set to: ");
  
}

void loop() {
}

void get_val(void* pvParameters)
{
  t1 = millis();
  (void) pvParameters;
  xSemaphoreTake(xangle,portMAX_DELAY);
  for(;;){
    mpu.getEvent(&a, &g, &temp);
    angle = RAD_TO_DEG * (atan2(-a.acceleration.y,-a.acceleration.x)*PI);
    //Serial.println("angle");
    x = fabs(angle);
    y = fabs(init_ang);
    z = x - y;
    check = fabs(z);
    Serial.println(check);
    if(check > 100)
    {
      Serial.println("in check");
      xSemaphoreGive(xalarm);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      xSemaphoreTake(xangle,portMAX_DELAY);
      t2 = millis();
      check = t2-t1;
      Serial.println("Total Time to complete process");
      Serial.println(check);
      noTone(buzzer);
    }
  }
}

void alarm(void* pvParameters)
{
  (void) pvParameters;
  for(;;){
   if (xSemaphoreTake(xalarm, portMAX_DELAY) == pdPASS){
      xSemaphoreTake(xalarm, portMAX_DELAY);
      Serial.println("alarm");
      tone(buzzer,1000);
      digitalWrite(LED,HIGH);
      digitalWrite(vib_dsc,HIGH);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      digitalWrite(LED,LOW);
      digitalWrite(vib_dsc,LOW);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      //xSemaphoreGive(xangle);
    } 
//    else xSemaphoreGive(x);
  }
}


/*interrupt to set a new init_angle*/
void Init_ISR(){
  noInterrupts();
  digitalWrite(7,HIGH);
  init_ang = angle;
  interrupts();
  xSemaphoreGive(xangle);
}

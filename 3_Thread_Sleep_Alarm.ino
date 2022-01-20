#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <math.h>

const uint8_t LED = 13;
const uint8_t interruptpin = 2;

Adafruit_MPU6050 mpu;

SemaphoreHandle_t xalarm;
SemaphoreHandle_t xangle;
SemaphoreHandle_t gangle;

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
  pinMode(interruptpin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptpin),Init_ISR,CHANGE);
  
  
  xTaskCreate(get_val,  "angle", 128, NULL, 0, NULL );
  xTaskCreate(alarm,  "alarm", 128, NULL, 0, NULL );
  xTaskCreate(get_angle,  "angle", 128, NULL, 0, NULL );

  gangle = xSemaphoreCreateBinary();
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

void get_angle(void* pvParameters)
{
  //xSemaphoreTake(xalarm,portMAX_DELAY);
  t1 = millis();
  (void) pvParameters;
  for(;;){
    xSemaphoreTake(gangle,portMAX_DELAY);
    mpu.getEvent(&a, &g, &temp);
    angle = RAD_TO_DEG * (atan2(-a.acceleration.y,-a.acceleration.x)*PI);
    xSemaphoreGive(xangle);
    t1 = millis();
  }
}

void get_val(void* pvParameters)
{
  (void) pvParameters;
  xSemaphoreTake(xangle,portMAX_DELAY);
  for(;;){
    x = fabs(angle);
    y = fabs(init_ang);
    z = x - y;
    check = fabs(z);
    Serial.println(check);
    if(check > 1)
    {
      Serial.println("in check");
      digitalWrite(8,HIGH);
      xSemaphoreGive(xalarm);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      xSemaphoreTake(xangle,portMAX_DELAY);
      t2 = millis();
      check = t1-t2;
      Serial.println("Total Time for Function");
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
      digitalWrite(LED,HIGH);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      digitalWrite(LED,LOW);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      tone(buzzer,1000);
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
  xSemaphoreGiveFromISR(gangle,NULL);
}

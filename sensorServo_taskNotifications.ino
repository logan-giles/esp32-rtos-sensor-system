#include <ESP32Servo.h>
#include <math.h>
#include <LiquidCrystal.h>

//Sensor Pins
#define ECHO_PIN 18
#define TRIG_PIN 5
#define SERVO_PIN 19

//Buzzer and RGB Pins
#define BUZZER 21
#define R_PIN 26
#define G_PIN 25
#define B_PIN 27

//Exponential Smoothing
#define ALPHA .2

//LCD Screen Pins
#define E_PIN 33
#define D0_PIN 32
#define D1_PIN 35
#define D2_PIN 34
#define D3_PIN 2
#define D4_PIN 4
#define D5_PIN 22
#define D6_PIN 23
#define D7_PIN 15
#define RS_PIN 13


//Servo declarations
Servo s;
const float inMIN = 2.0f;
const float inMAX = 70.0f;
const float outMIN = 15.0f;
const float outMAX = 165.0f;
float previousAngle = 90.0f;
float targetAngle;
float filteredDistance = -2.0f;

//LCD Declarations
uint32_t lastPrintTime = 0;
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

//Task Handles
TaskHandle_t servoTaskHandle;
TaskHandle_t serialTaskHandle;
TaskHandle_t stateIndicateTaskHandle;
TaskHandle_t lcdTaskHandle;

// STATE ENUM
typedef enum STATE {SAFE, CAUTION, DANGER, IGNORE} state;
 
//Queue Struct
  typedef struct state_Distance {
    state system_state;
    float distance;
  } sensorPacket;

//Queue Handle
QueueHandle_t sensorData;

//RGB
const int R_CH = 0, G_CH = 1, B_CH = 2;

//Utility Functions
void ledOff(){
  digitalWrite(R_PIN, LOW);
  digitalWrite(B_PIN, LOW);
  digitalWrite(G_PIN, LOW);
}

float clamp(const float inMIN, const float inMAX, const float distance){
  if(distance > inMAX) return inMAX;
  if(distance < inMIN) return inMIN;
  return distance;
}
float servoMap(const float inMIN, const float inMAX, const float outMIN, const float outMAX, const float distance){
  return (((distance - inMIN) / (inMAX - inMIN)) * (outMAX - outMIN) + outMIN);
}

//Task Functions
void sensorTask(void *pvParameters) {
  TickType_t xLastWakeTime = pdMS_TO_TICKS(0);

  while(1){
    const TickType_t xFrequency = pdMS_TO_TICKS(80);
    xLastWakeTime = xTaskGetTickCount();

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    sensorPacket queueData;
    queueData.distance = (pulseIn(ECHO_PIN, HIGH, 30000) /29.155) / 2;
    
    if(queueData.distance == 0){
      queueData.system_state = IGNORE;
    } else if(queueData.distance < 15 && queueData.distance > 0){
      queueData.system_state = DANGER;
    } else if(queueData.distance < 30){
      queueData.system_state = CAUTION;
    } else {
      queueData.system_state = SAFE;
    }
    xQueueOverwrite(sensorData, &queueData);

    // Notifications
    xTaskNotifyGive(servoTaskHandle);
    xTaskNotifyGive(serialTaskHandle);
    xTaskNotifyGive(stateIndicateTaskHandle);
    xTaskNotifyGive(lcdTaskHandle);


    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void serialTask(void *pvParameters){

  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    sensorPacket serialData;
    if(xQueuePeek(sensorData, &serialData, pdMS_TO_TICKS(100)) == pdTRUE){


      Serial.print("Distance: ");
      Serial.println(serialData.distance);
     switch(serialData.system_state){
        case SAFE:
          Serial.println("System state: SAFE");
          break;
        case CAUTION:
         Serial.println("System state: CAUTION");
         break;
       case DANGER:
         Serial.println("System state: DANGER");
         break;
        case IGNORE:
          Serial.println("NO READING");
          break;
      }
    }
    
  }
}

void stateIndicateTask(void *pvParameters){
  sensorPacket stateIndicateData;



  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(xQueuePeek(sensorData, &stateIndicateData, pdMS_TO_TICKS(100)) == pdTRUE){
      if(stateIndicateData.system_state == SAFE){
        ledOff();
        digitalWrite(G_PIN, HIGH);
        digitalWrite(BUZZER, LOW);
      } else if(stateIndicateData.system_state == CAUTION){
        ledOff();
        digitalWrite(R_PIN, HIGH);
        digitalWrite(G_PIN, HIGH);
        digitalWrite(BUZZER, LOW);
      } else if(stateIndicateData.system_state == DANGER){
        ledOff();
        digitalWrite(R_PIN, HIGH);
        digitalWrite(BUZZER, HIGH);
      } else if(stateIndicateData.system_state == IGNORE){
        ledOff();
        digitalWrite(G_PIN, HIGH);
        digitalWrite(BUZZER, LOW);
      }
    }
    
  }
}

void servoTask(void * pvParameters){

  while(1){
    sensorPacket servoPacket;

    if(xQueuePeek(sensorData, &servoPacket, pdMS_TO_TICKS(100)) == pdTRUE){
      if(servoPacket.system_state == IGNORE){ //If pulseIn returns 0, do nothing 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        continue;
      }

      servoPacket.distance = clamp(inMIN, inMAX, servoPacket.distance);

      if(filteredDistance == -2) {
        filteredDistance = servoPacket.distance;
      } else {
        filteredDistance = filteredDistance + ALPHA * (servoPacket.distance - filteredDistance);
      }

      targetAngle = servoMap(inMIN, inMAX, outMIN, outMAX, filteredDistance);

      if(fabs(targetAngle - previousAngle) > 3.0f){
        s.write((int)targetAngle);
        previousAngle = targetAngle;
      }

      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
  }
}

void lcdTask(void * pvParameters){
  sensorPacket lcdPacket;
  lcd.setCursor(0,0);
  lcd.print("Dist(cm): ");
  lcd.setCursor(0,1);
  lcd.print("State: ");

  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(xQueuePeek(sensorData, &lcdPacket, pdMS_TO_TICKS(100)) == pdTRUE){
      
      if((millis() - lastPrintTime) >= 500){

        lcd.setCursor(11, 0);

        if(lcdPacket.distance < 10){
          lcd.print("  ");
          lcd.print(lcdPacket.distance, 0);
        }else if(lcdPacket.distance < 100){
            lcd.print(" ");
            lcd.print(lcdPacket.distance, 0);
        } else{
            lcd.print(lcdPacket.distance, 0);
          }
        
        lcd.setCursor(8, 1);
        if(lcdPacket.system_state == SAFE){
          lcd.print("SAFE   ");
        } else if(lcdPacket.system_state == CAUTION){
          lcd.print("CAUTION");
        } else if(lcdPacket.system_state == DANGER){
          lcd.print("DANGER ");
        } else if(lcdPacket.system_state == IGNORE){
          lcd.print("IGNORE ");
        }
          lastPrintTime = millis();
          
        }
      }
    }
  }



void setup() {
  Serial.begin(115200);
  //Pin setups
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  //Servo Setup
  s.attach(SERVO_PIN, 500, 2500);
  s.setPeriodHertz(50);
  s.write(90);

  //LCD Setup
  lcd.begin(16,2);

  //Queue Create
  sensorData = xQueueCreate(1, sizeof(sensorPacket));
  if(sensorData == NULL){
    Serial.println("SENSOR QUEUE IS NULL");
    delay(200000);
  }
  

  //Task Creation
  xTaskCreate(sensorTask, "Sensor", 4800, NULL, 1, NULL);
  xTaskCreate(servoTask, "Servo", 4800, NULL, 2, &servoTaskHandle);
  xTaskCreate(stateIndicateTask, "stateIndicate", 2400, NULL, 2, &stateIndicateTaskHandle);
  xTaskCreate(serialTask, "Serial", 2400, NULL, 1, &serialTaskHandle);
  xTaskCreate(lcdTask, "LCD", 2400, NULL, 1, &lcdTaskHandle);
}

void loop() {
  // put your main code here, to run repeatedly:

}

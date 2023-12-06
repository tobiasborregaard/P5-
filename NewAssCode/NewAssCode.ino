#include <Arduino.h>
#include <ESP32Servo.h>

Servo myservo;

// =================== PINS ==================
/*
  Programmet crasher hvis vi prøver at bruge pinMode på en pin der ikke eksisterer.
  Derfor er det vigtigt at de her pins har de rigtige værdier!
*/
// Sound Sensor L
const int triggerPinL = 7;
const int echoPinL = 5;
// Sound Sensor C
const int triggerPinC = 12;
const int echoPinC = 3;
// Sound Sensor R
const int triggerPinR = 8;
const int echoPinR = 6;
// Servo
const int servoPin = 32;
// HBRO
const int hBro_PWM = 39;
const int hBro_pol1 = 20;
const int hBro_pol2 = 21;

// FIR filter order
const int M = 10;

// Global variables
double duration1, distance1, duration2, distance2, duration3, distance3, avgDist, relAng;

// Separation between sensors in m
const double separationLR = 0.185;  // Separation between left and right sensor
const double separationRC = 0.098;  // Separation between center and right sensor
const double separationLC = 0.098;  // Separation between center and left sensor

// Semaphores
static SemaphoreHandle_t angMutex;
static SemaphoreHandle_t distMutex;
static SemaphoreHandle_t printMutex;
static SemaphoreHandle_t orderMutex;

void setup() {
  Serial.begin(115200);
  
  myservo.attach(servoPin);

  /*
  //Distance Sensor Init
  pinMode(triggerPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(triggerPinC, OUTPUT);
  pinMode(echoPinC, INPUT);
  pinMode(triggerPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  //H-Bridge Init
  pinMode(hBro_PWM, OUTPUT);
  pinMode(hBro_pol1, OUTPUT);
  pinMode(hBro_pol2, OUTPUT);
  */
  
  angMutex = xSemaphoreCreateMutex();
  distMutex = xSemaphoreCreateMutex();
  printMutex = xSemaphoreCreateMutex();
  orderMutex = xSemaphoreCreateMutex();

  xTaskCreate(Angle_Controller, "Angle controller", 3000, NULL, 3, NULL);
  xTaskCreate(Dist_Controller, "Distance controller", 3000, NULL, 2, NULL);
  xTaskCreate(UniTask, "UniTask", 3000, NULL, 1, NULL);
}

void loop() {
  // Empty loop since the scheduler will handle the task execution
}

void Angle_Controller(void *pvParameters) {
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  // Initialize values
  double angOP = 1011.44;
  double out = 0;

  //Input
  double Ea_n0 = 0;
  //Output
  double Ta_n0 = 0;
  //Ref
  double ref = 0;
  

  while (1) {
    xSemaphoreTake(orderMutex, 1000);

    xSemaphoreTake(printMutex, 1000);
    Serial.print("Angle: ");
    Serial.println(millis());
    xSemaphoreGive(printMutex);
    
    xSemaphoreTake(angMutex, 1000);
    Ea_n0 = ref + relAng;
    xSemaphoreGive(angMutex);

    Ta_n0 = 16.86*Ea_n0 + angOP;

    if(Ta_n0 > 1429.0) {
      out = 1429.0;
    }
    else if(Ta_n0 < 594) {
      out = 594;
    }
    else {
      out = Ta_n0;
    }

    myservo.writeMicroseconds(out);
    
    xSemaphoreGive(orderMutex);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
  }
}

void Dist_Controller(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  double distOP = 0.276;
  double out = 0;
  int outMapped = 0;
  int dir = 1;

  // Initialize values
  //Input
  double Ed_n0 = 0;
  double Ed_n1 = 0;
  double Ed_n2 = 0;
  //Output
  double Vd_n0 = 0;
  double Vd_n1 = 0;
  double Vd_n2 = 0;
  //Ref
  double ref = -0.7;
  
  while (1) {
    
    xSemaphoreTake(orderMutex, 1000);

    xSemaphoreTake(printMutex, 1000);
    Serial.print("Distance: ");
    Serial.println(millis());
    xSemaphoreGive(printMutex);

    xSemaphoreTake(distMutex, 1000);
    Ed_n0 = ref + avgDist;
    xSemaphoreGive(distMutex);
    Vd_n0 = 99 * Ed_n0 - 198 * Ed_n1 + 98 * Ed_n2 + 1.85 * Vd_n1 - 0.85 * Vd_n2 + distOP;

    if(Vd_n0 < 0) {
      dir = 0;
      if(Vd_n0 < -7.0) {
        out = 7.0;
      }
      else {
        out = Vd_n0;
      }
    }
    else {
      dir = 1;
      if(Vd_n0 > 7.0) {
        out = 7.0;
      }
      else {
        out = Vd_n0;
      }
    }

    int outMapped = mapFloatToInt(out, 0.0, 7.0, 0, 255);

    if(dir) {
      //digitalWrite(hBro_pol1, HIGH);
      //digitalWrite(hBro_pol2, LOW);
    }
    else {
      //digitalWrite(hBro_pol2, HIGH);
      //digitalWrite(hBro_pol1, LOW);
    }

    //digitalWrite(hBro_PWM, outMapped);
    
    xSemaphoreGive(orderMutex);
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void UniTask(void *pvParameters) {
  (void)pvParameters;

  // Arrays for storing the distance measurements for each sensor
  double measL[M + 1] = { 0 }, measC[M + 1] = { 0 }, measR[M + 1] = { 0 };
  // Variables for the sum of the measurements
  double sumL = 0,  sumC = 0,  sumR = 0;
  int index = 0;

  // Initialize variables
  double filteredDistL = 0;
  double filteredDistC = 0;
  double filteredDistR = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    xSemaphoreTake(printMutex, 1000);
    Serial.print("UNI: ");
    Serial.println(millis());
    xSemaphoreGive(printMutex);

    //filteredDistL = ReadAndFilterSensor(triggerPinL, echoPinL, measL, sumL, index);  // Measured and filtered distance from sensorL
    filteredDistL = 0.7;  // dummy
    //vTaskDelay(pdMS_TO_TICKS(5));

    //filteredDistC = ReadAndFilterSensor(triggerPinC, echoPinC, measC, sumC, index);  // Measured and filtered distance from sensorC
    filteredDistC = 0.7;  // dummy
    //vTaskDelay(pdMS_TO_TICKS(5));

    //filteredDistR = ReadAndFilterSensor(triggerPinR, echoPinR, measR, sumR, index); // Measured and filtered distance from sensorR
    filteredDistR = 0.7;  // dummy

    //Serial.print("Sensor L: ");
    //Serial.println(filteredDistL, 4);
    //Serial.print("Sensor C: ");
    //Serial.println(filteredDistC, 4);
    //Serial.print("Sensor R: ");
    //Serial.println(filteredDistR, 4);

    xSemaphoreTake(angMutex, 1000);
    avgDist = AvgDistance(filteredDistL, filteredDistC, filteredDistR);
    xSemaphoreGive(angMutex);
    //Serial.print("Average Distance: ");
    //Serial.println(avgDist, 4);

    xSemaphoreTake(distMutex, 1000);
    relAng = RelAngle(filteredDistL, filteredDistC, filteredDistR);
    xSemaphoreGive(distMutex);
    //Serial.print("Relative Angle: ");
    //Serial.println(relAng, 2);


    index = (index + 1) % (M + 1);  // increase index and wrap around if necessary
    //Serial.println("");


    vTaskDelayUntil(&xLastWakeTime, 10);
  }
}

double ReadAndFilterSensor(int triggerPin, int echoPin, double *measurements, double &sum, int index) {
  unsigned long startTime;
  long duration;
  double distance;

  digitalWrite(triggerPin, LOW);

  digitalWrite(triggerPin, HIGH);
  startTime = micros();
  vTaskDelay(pdMS_TO_TICKS(5));
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2.0) / 2911.0;  // Convert to distance

  sum -= measurements[index];
  measurements[index] = distance;
  sum += measurements[index];

  return sum / (M + 1);
}


double AvgDistance(double filteredDistL, double filteredDistC, double filteredDistR) {
  if (filteredDistL < 1.5 && filteredDistR < 1.5) {
    //Serial.println("Distance_Case_1");
    return (filteredDistL + filteredDistR) / 2;
  }

  if (filteredDistL > 1.5) {
    //Serial.println("Distance_Case_2");
    return (filteredDistC + filteredDistR) / 2;
  }

  if (filteredDistR > 1.5) {
    //Serial.println("Distance_Case_3");
    return (filteredDistL + filteredDistC) / 2;
  }
}



double RelAngle(double filteredDistL, double filteredDistC, double filteredDistR) {
  if (filteredDistL < 1.5 && filteredDistR < 1.5) {
    //Serial.println("Angle_Case_1");
    return atan(abs(filteredDistL - filteredDistR) / separationLR) * (180 / 3.14);
  }

  if (filteredDistL > 1.5) {
    //Serial.println("Angle_Case_2");
    return atan(abs(filteredDistC - filteredDistR) / separationRC) * (180 / 3.14);
  }

  if (filteredDistR > 1.5) {
    //Serial.println("Angle_Case_3");
    return atan(abs(filteredDistL - filteredDistC) / separationLC) * (180 / 3.14);
  }
}

int mapFloatToInt(double x, double in_min, double in_max, int out_min, int out_max) {
    return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

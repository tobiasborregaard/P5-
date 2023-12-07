#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

Servo myservo;

// =================== PINS ==================
/*
  Programmet crasher hvis vi prøver at bruge pinMode på en pin der ikke eksisterer.
  Derfor er det vigtigt at de her pins har de rigtige værdier!
*/
// Sound Sensor L
const int triggerPinL = 32;
const int echoPinL = 34;
// Sound Sensor C
const int triggerPinC = 33;
const int echoPinC = 35;
// Sound Sensor R
const int triggerPinR = 27;
const int echoPinR = 25;
// Servo
const int servoPin = 26;
// HBRO
const int hBro_PWM = 14;
const int hBro_pol1 = 22;
const int hBro_pol2 = 23;

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

// =================== ESP-NOW ==================
//Mac addresses // 08:3A:F2:45:44:BC
// 7C:DF:A1:1A:57:66
// 94:B5:55:F9:06:44
uint8_t peerAddress[] = { 0x7C, 0xDF, 0xA1, 0x1A, 0x57, 0x66 };

#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
struct message {
  double value;
} __attribute__((packed));

void setup() {
  Serial.begin(230400);

  myservo.attach(servoPin);
  
  Serial.println("Printing Mac");
  InitESP(); //ESPNOW init

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

  angMutex = xSemaphoreCreateMutex();
  distMutex = xSemaphoreCreateMutex();
  printMutex = xSemaphoreCreateMutex();
  orderMutex = xSemaphoreCreateMutex();

  xTaskCreate(Angle_Controller, "Angle controller", 4000, NULL, 3, NULL);
  xTaskCreate(Dist_Controller, "Distance controller", 4000, NULL, 2, NULL);
  xTaskCreate(UniTask, "UniTask", 4000, NULL, 1, NULL);
}

void loop() {
  // Empty loop since the scheduler will handle the task execution
}

void Angle_Controller(void *pvParameters) {
  (void)pvParameters;
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
    //Serial.print("Angle: ");
    //Serial.println(millis());
    xSemaphoreGive(printMutex);

    xSemaphoreTake(angMutex, 1000);
    Ea_n0 = ref + relAng;
    xSemaphoreGive(angMutex);

    //Ta_n0 = 16.86 * Ea_n0 + angOP;
    Ta_n0 = 1 * Ea_n0 + angOP;

    if (Ta_n0 > 1429.0) {
      out = 1429.0;
    } else if (Ta_n0 < 594.0) {
      out = 594.0;
    } else {
      out = Ta_n0;
    }

    myservo.writeMicroseconds(out);

    xSemaphoreGive(orderMutex);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(25));
  }
}

void Dist_Controller(void *pvParameters) {
  (void)pvParameters;
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
    //Serial.println();
    //Serial.print("Distance: ");
    //Serial.println(millis());
    
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    xSemaphoreGive(printMutex);

    xSemaphoreTake(distMutex, 1000);
    Ed_n0 = ref + avgDist;
    xSemaphoreGive(distMutex);

    //wc=3, PM = 75, SSE = 0.01, Rise time = 0.49s, MP = 5.67%,
    //Vd_n0 = 89.9*Ed_n0 - 178.4*Ed_n1 + 88.5*Ed_n2 + 1.67*Vd_n1 - 0.672*Vd_n2;

    //wc=2, PM = 75, SSE = 0.01, Rise time = 0.75s, MP = 4.07%,
    //Vd_n0 = 37.41*Ed_n0 - 74.36*Ed_n1 + 36.96*Ed_n2 + 1.797*Vd_n1 - 0.7967*Vd_n2;

    //wc=1, PM = 65, SSE = 0.01, Rise time = 1.36s, MP = 5.77%, (god)
    Vd_n0 = 5.721 * Ed_n0 - 11.38 * Ed_n1 + 5.66 * Ed_n2 + 1.94 * Vd_n1 - 0.94 * Vd_n2;


    //wc=3, PM = 75, SSE = 0.1, Rise time = 0.5s, MP = 4.74%,
    //Vd_n0 = 89.88*Ed_n0 - 178.4*Ed_n1 + 88.5*Ed_n2 + 1.67*Vd_n1 - 0.672*Vd_n2;

    //wc=2, PM = 75, SSE = 0.1, Rise time = 0.76s, MP = 3.66%,
    //Vd_n0 = 37.4*Ed_n0 - 74.36*Ed_n1 + 36.96*Ed_n2 + 1.796*Vd_n1 - 0.797*Vd_n2;

    //wc=1, PM = 65, SSE = 0.114, Rise time = 1.38s, MP = 4.8%,
    //Vd_n0 = 5.095*Ed_n0 - 10.13*Ed_n1 + 5.032*Ed_n2 + 1.947*Vd_n1 - 0.9468*Vd_n2;

    //Serial.print("Vd_n0: ");
    //Serial.println(Vd_n0);
    // Update variables
    Vd_n2 = Vd_n1;
    Vd_n1 = Vd_n0;
    Ed_n2 = Ed_n1;
    Ed_n1 = Ed_n0;

    xSemaphoreTake(printMutex, 1000);
    //Serial.print("out: ");
    //Serial.println(out);
    xSemaphoreGive(printMutex);
    if (Vd_n0 < 0.0) {
      dir = 0;
      //out = 0.0;

      if ((Vd_n0 < -7.0) && ((Vd_n0 - distOP) < -7.0)) {
        out = 7.0;
      } else {
        out = -Vd_n0 + distOP;
      }

    } else {
      dir = 1;
      if ((Vd_n0 > 7.0) && ((Vd_n0 + distOP) > 7.0)) {
        out = 7.0;
      } else {
        out = Vd_n0 + distOP;
      }
    }
    xSemaphoreTake(printMutex, 1000);
    //Serial.print("out: ");
    //Serial.println(out);
    xSemaphoreGive(printMutex);

    int outMapped = mapDoubleToInt(out, 0.0, 7.0, 0, 255);

    xSemaphoreTake(printMutex, 1000);
    //Serial.print("dir: ");
    //Serial.println(dir);
    xSemaphoreGive(printMutex);

    if (dir) {
      digitalWrite(hBro_pol1, HIGH);
      digitalWrite(hBro_pol2, LOW);
    } else {
      digitalWrite(hBro_pol2, HIGH);
      digitalWrite(hBro_pol1, LOW);
    }

    xSemaphoreTake(printMutex, 1000);
    Serial.print("Outmapped: ");
    Serial.println(outMapped);
    xSemaphoreGive(printMutex);

    if ((outMapped < 40) && (outMapped > 10)) {
      analogWrite(hBro_PWM, 40);
      vTaskDelay(pdMS_TO_TICKS(2));
      analogWrite(hBro_PWM, outMapped);
    } else if (outMapped < 10) {
      analogWrite(hBro_PWM, 0);
    } else {
      analogWrite(hBro_PWM, outMapped);
    }


    xSemaphoreGive(orderMutex);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));
  }
}

void UniTask(void *pvParameters) {
  (void)pvParameters;

  // Arrays for storing the distance measurements for each sensor
  double measL[M + 1] = { 0 }, measC[M + 1] = { 0 }, measR[M + 1] = { 0 };
  // Variables for the sum of the measurements
  double sumL = 0, sumC = 0, sumR = 0;
  int index = 0;

  // Initialize variables
  double filteredDistL = 0;
  double filteredDistC = 0;
  double filteredDistR = 0;

  bool validL, validC, validR;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    xSemaphoreTake(printMutex, 1000);
    //Serial.println();
    //Serial.print("UNI: ");
    //Serial.println(millis());
    xSemaphoreGive(printMutex);

    filteredDistL = ReadAndFilterSensor(triggerPinL, echoPinL, measL, sumL, index, validL);  // Measured and filtered distance from sensorL
    //filteredDistL = 0.7;  // dummy
    //vTaskDelay(pdMS_TO_TICKS(5));

    filteredDistC = ReadAndFilterSensor(triggerPinC, echoPinC, measC, sumC, index, validC);  // Measured and filtered distance from sensorC
    //filteredDistC = 0.7;  // dummy
    //vTaskDelay(pdMS_TO_TICKS(5));

    filteredDistR = ReadAndFilterSensor(triggerPinR, echoPinR, measR, sumR, index, validR);  // Measured and filtered distance from sensorR
    //filteredDistR = 0.7;  // dummy
    
    Serial.print("Sensor L: ");
    Serial.println(filteredDistL);
    Serial.print("Sensor C: ");
    Serial.println(filteredDistC);
    Serial.print("Sensor R: ");
    Serial.println(filteredDistR);
    

    xSemaphoreTake(angMutex, 1000);
    avgDist = AvgDistance(validL, validC, validR, filteredDistL, filteredDistC, filteredDistR);
    xSemaphoreGive(angMutex); /*
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();*/
    Serial.print("Average Distance: ");
    Serial.println(avgDist, 4);


    xSemaphoreTake(distMutex, 1000);
    relAng = RelAngle(validL, validC, validR, filteredDistL, filteredDistC, filteredDistR);
    xSemaphoreGive(distMutex);
    Serial.print("Relative Angle: ");
    Serial.println(relAng, 2);


    index = (index + 1) % (M + 1);  // increase index and wrap around if necessary
    //Serial.println("");


    vTaskDelayUntil(&xLastWakeTime, 25);
  }
}

double ReadAndFilterSensor(int triggerPin, int echoPin, double *measurements, double &sum, int index, bool &valid) {
  unsigned long startTime;
  long duration;
  double distance;
  double timeOut = 7000;  // Timeout in microseconds - corresponding to 1.2 meter (7000 µs)

  digitalWrite(triggerPin, LOW);

  digitalWrite(triggerPin, HIGH);
  startTime = micros();
  while (micros() - startTime < 10);
  digitalWrite(triggerPin, LOW);

  noInterrupts();
  duration = pulseIn(echoPin, HIGH, timeOut);
  interrupts();
  //Serial.println(duration);

  // Calculate distance or assign max value if timeout
  if (duration == 0) {
    distance = 1.201;  // Assign 1.2 meters if timeout occurs
  } else {
    distance = (duration / 2.0) / 2911.0;  // Calculate actual distance
  }
  //distance = (duration / 2.0) / 2911.0;


  sum -= measurements[index];
  measurements[index] = distance;
  sum += measurements[index];
  double filteredValue = sum / (M + 1);

  valid = (filteredValue <= 1.2);
  return sum / (M + 1);
}


double AvgDistance(bool validL, bool validC, bool validR, double filteredDistL, double filteredDistC, double filteredDistR) {
  // When both left and right sensors are valid
  if (validL && validR) {
    Serial.println("Case LR");
    return (filteredDistL + filteredDistR) / 2;
  }

  // When left is invalid, but right and center are valid
  if (!validL && validR && validC) {
    Serial.println("Case CR");
    return (filteredDistC + filteredDistR) / 2;
  }

  // When right is invalid, but left and center are valid
  if (validL && !validR && validC) {
    Serial.println("Case LC");
    return (filteredDistL + filteredDistC) / 2;
  }

  // When only left is valid
  if (validL && !validC && !validR) {
    Serial.println("Case L");
    return filteredDistL;
  }

  // When only right is valid
  if (!validL && !validC && validR) {
    Serial.println("Case R");
    return filteredDistR;
  }

  // When only center is valid
  if (!validL && validC && !validR) {
    Serial.println("Case C");
    return filteredDistC;
  }

  // Default case when none of the sensors are valid
  Serial.println("Case Else");
  return (filteredDistL + filteredDistR) / 2;  // This could be modified to handle the "all invalid" case differently
}

double RelAngle(bool validL, bool validC, bool validR, double filteredDistL, double filteredDistC, double filteredDistR) {
  if (validL && validR) {
    return (double)atan(((float)filteredDistL - (float)filteredDistR) / (float)separationLR) * (180.0 / 3.14);
  }

  if (!validL && validC && validR) {
    return (double)atan(((float)filteredDistC - (float)filteredDistR) / (float)separationRC) * (180.0 / 3.14);
  }

  if (validL && validC && !validR) {
    return (double)atan(((float)filteredDistL - (float)filteredDistC) / (float)separationLC) * (180.0 / 3.14);
  }

  if (!validL && !validC && validR) {
    return 25.0; // max angle right
  }

  if (validL && !validC && !validR) {
    return -25.0; // max angle left
  }

  else {
    return 0.0;  // continue to drive forward
  }
}

int mapDoubleToInt(double x, double in_min, double in_max, int out_min, int out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// ESP-NOW
void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  // Initialize ESP-NOW, restart on failure
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;

  // Add peer, return if failed
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return;
  }

  // Register callback functions for receiving and sending data
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  message *msg = (message *)incomingData;/*
  Serial.println();
  Serial.print("Message received: ");
  Serial.println(msg->value);
  Serial.println();*/
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

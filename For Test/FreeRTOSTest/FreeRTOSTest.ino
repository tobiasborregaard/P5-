#include <Arduino_FreeRTOS.h>
#include <semphr.h>        //Denne header fil er afgørende for deklarering af semaphorer!!!

TickType_t TimeStart;      // used for timetaking
TickType_t TimeEnd;        // used for timetaking
TickType_t xLastWakeTime;  // used for timing

SemaphoreHandle_t mutSem = xSemaphoreCreateMutex(); // Mutex for critical regions

SemaphoreHandle_t Ang = xSemaphoreCreateBinary();   // Binær semafor til at starte angle controlleren
SemaphoreHandle_t DIST = xSemaphoreCreateBinary();  // binary semaphore til at starte distance controlleren
SemaphoreHandle_t UNI = xSemaphoreCreateBinary();   // binary semaphore til start af Universal funktion / feedback Funktion



//=================== Variabler der relaterer sig til Uni funktion ==================
// Sound Sensor 1
const int triggerPin1 = 8;  // Ultrasonic sensor trigger pin
const int echoPin1 = 6;     // Ultrasonic sensor echo pin

// Sound Sensor 2
const int triggerPin2 = 7;  // Ultrasonic sensor trigger pin
const int echoPin2 = 5;     // Ultrasonic sensor echo pin

// Sound Sensor 3
const int triggerPin3 = 12;  // Ultrasonic sensor trigger pin
const int echoPin3 = 3;      // Ultrasonic sensor echo pin

double duration1, distance1, duration2, distance2, duration3, distance3, avgDist, RelAngle;

float Kat2 = 9.8;    //Korteste afstand mellem sensorerne 
float Kat1 = 18.5;   //Længste Afstand mellem ultralydssensorerne



//=================== Relaterer sig til configuration af kerner =====================
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif


void setup() {
  Serial.begin(115200);

  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  xSemaphoreGive(mutSem);      // Sikrer at mutSem is til rådighed
  xSemaphoreGive(DIST);        // Sikrer at Semaphor er til rådighed
  xSemaphoreGive(Ang);         // Sikrer at Semaphor er til rådighed
  xSemaphoreGive(UNI);         // Sikrer at Semaphor er til rådighed


  // ==================== Task creation ====================
  xTaskCreate(Angle_Controller, "Angle controller",    256,  NULL,  1, NULL);
  xTaskCreate(Dist_Controller,  "Distance controller", 256,  NULL,  1, NULL);
  xTaskCreate(Uni_Function,     "Feedback Funktion",   256,  NULL,  1, NULL);

  //Oprettelse af task på en specifik kerne: core 0 - PRO  core 1 - APP
  //xTaskCreatePinnedToCore(Toggle_LED1, "RØD LED", 256, NULL, 1, NULL, app_cpu);

  vTaskStartScheduler(); // Starter FreeRTOS scheduler
}








// ======================================== Angle controller TASK ======================================
void Angle_Controller(void *Parameter) {


  xLastWakeTime = xTaskGetTickCount();        // Retunerer antallet af ticks som er passeret siden Scheduleren er blevet kaldt.

  while (1) {
    xSemaphoreTake(Ang, portMAX_DELAY);       // Acquires semaphore. Waits infinitely!
    //xSemaphoreTake(mutSem, portMAX_DELAY);  // Acquires mutex. Waits infinitely!


    // Her indsættes Emils proportional controller  @ y[n] = Kp * x[n]?
    // Referencen til angle controlleren er nul og oprettes blot ved en local eller variabel.
    // ift. feedback (Vinklen mellem bilerne) kan vinklen tilgåes ved den globale variabel "RelAngle" fra uniFunc
    // Ift. feedforward (Leadercar hjul vinkel) kan den tilgåes ved den globale variabel U som fra ESP now funktion


    //xSemaphoreGive(mutSem);                                    // Release af mutex semaphore
    //vTaskDelayUntil( &xLastWakeTime, X / portTICK_PERIOD_MS)   // Sikrer at tasken venter X ms tid, ift. den nuværende tid ("&xLastWakeTime") @ Periode Tid def.
  }
  xSemaphoreGive(DIST); // Vidergivere semafor - Skal nok ikke anvendes - Vi bruger TaskDelay funktionen for at holde det periodisk, 
}





// ======================================== Distance controller TASK ======================================
void Dist_Controller(void *Parameter) {

  xLastWakeTime = xTaskGetTickCount();        // Retunerer antallet af ticks som er passeret siden Scheduleren er blevet kaldt.

  while (1) {
    xSemaphoreTake(DIST, portMAX_DELAY);
    //xSemaphoreTake(mutSem, portMAX_DELAY);


    // Her indsættes differenceligningen for Distance controller og alt relateret til denne.
    // Referencen er vores safety distance på 0.7m
    // Husk at indsætte task prioritet i linje 67.
    // Ift. Feedback (Afstanden mellem bilerne) anvendes den globale variabel "avgDist" fra UniFunc.
    // ift. Feedforward (Leader car hastighed) kan den tilgåes ved den globale variabel Y.
    // Outputtet af controlleren er en spænding som evt. skal konverteres til et PWM signal som skal videre til H-broen og til DC motoren (Plant)

    //xSemaphoreGive(mutSem);                                    // Release af mutex semaphore
    //vTaskDelayUntil( &xLastWakeTime, X / portTICK_PERIOD_MS)   // Sikrer at tasken venter X ms tid, ift. den nuværende tid ("&xLastWakeTime") @ Periode Tid def.
  }
  xSemaphoreGive(UNI);                                           // Aflevere nøglen så universal funktionen kører.
}








//==================== Universal funktion - Retunerer Afstand mellem Leader og Follower car samt den relative vinkel ===========================
//==================== Afstanden bruges som feedback til distance controlleren og vinklen bruges som feedback til angle controlleren ===========
//==================== Anvender 3 Ultra lyds sensorer til at bestemme afstanden samt den relative vinkel =======================================

void Uni_Function(void *Parameter) {

  xLastWakeTime = xTaskGetTickCount();        // Retunerer antallet af ticks som er passeret siden Scheduleren er blevet kaldt.

  while (1) {
    xSemaphoreTake(UNI, portMAX_DELAY);
    //xSemaphoreTake(mutSem, portMAX_DELAY);

    // Trigger Sensor 1
    digitalWrite(triggerPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin1, LOW);

    duration1 = pulseIn(echoPin1, HIGH);    // PulseIn retunerer længden af den incomming pulse i uS. Den venter på at echo pin går fra lav til høj og tager tid på hvor lang tid dette tager. 

    //Afstand i cm for sensor 1
    distance1 = (duration1 / 2) / 29.1;

    // Trigger Sensor 2
    digitalWrite(triggerPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin2, LOW);

    duration2 = pulseIn(echoPin2, HIGH);

    //Afstand i cm for sensor 2
    distance2 = (duration2 / 2) / 29.1;


    // Trigger Sensor 3
    digitalWrite(triggerPin3, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin3, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin3, LOW);

    duration3 = pulseIn(echoPin3, HIGH);

    //Afstand i cm for sensor 3
    distance3 = (duration3 / 2) / 29.1;

    // Print the distances to the serial monitor
    Serial.print("Distance Sensor 1: ");
    Serial.print(distance1);
    Serial.println(" cm");

    Serial.print("Distance Sensor 2: ");
    Serial.print(distance2);
    Serial.println(" cm");

    Serial.print("Distance Sensor 3: ");
    Serial.print(distance3);
    Serial.println(" cm");

    // Brug af de to yderste sensorer
    if (distance2 < 150 && distance1 < 150) {
      avgDist = (distance1 + distance2) / 2;           // ✡ Den gennemsnitslige afstand mellem leader og follower
      Serial.print("Average Distance ");
      Serial.print(avgDist);
      Serial.println(" cm ");

      float diff = distance1 - distance2;

      //Vinkel bestemmes
      RelAngle = atan(diff / Kat1) * 180 / 3.141596;    // ✡ Den relative vinkel mellem leader og follower
      Serial.print("Relative vinkel ");
      Serial.print(RelAngle);
      Serial.println(" °");
    }

    //Hvis den yderste sensorer ikke fanger bilen tages der udgangspunkt i senoren i midten
    if (distance2 > 150) {
      avgDist = (distance1 + distance3) / 2;
      Serial.print("Average Distance ");
      Serial.print(avgDist);
      Serial.println(" cm ");

      float diff = distance1 - distance3;

      //Vinkel bestemmes
      RelAngle = atan(diff / Kat2) * 180 / 3.141596;
      Serial.print("Relative vinkel ");
      Serial.print(RelAngle);
      Serial.println(" °");
    }

    //Hvis den yderste sensorer ikke fanger bilen tages der udgangspunkt i senoren i midten
    if (distance1 > 150) {
      avgDist = (distance2 + distance3) / 2;
      Serial.print("Average Distance ");
      Serial.print(avgDist);
      Serial.println(" cm ");

      float diff = distance2 - distance3;
      
      //Vinkel bestemmes
      RelAngle = atan(diff / Kat2) * 180 / 3.141596;
      Serial.print("Relative vinkel ");
      Serial.print(RelAngle);
      Serial.println(" °");

    }
    //xSemaphoreGive(mutSem);
    //vTaskDelayUntil( &xLastWakeTime, X / portTICK_PERIOD_MS)    // Sikrer at tasken venter X ms tid, ift. den nuværende tid ("&xLastWakeTime") @ Periode Tid def.
  }
  xSemaphoreGive(Ang);           //Aflevere til næste task?
}

void loop() {}

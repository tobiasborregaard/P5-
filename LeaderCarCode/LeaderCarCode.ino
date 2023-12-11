#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ############# PINS #############
const int ServoPin = 32;         // GPIO 4 on the ESP  //A1 pin på Arduino
float input_voltage = 0.0;

const int OpticPin = 39;  //pin 3 på arduino

double lasttimeDB = 0;
double delaytimer = 60;

double speedTimeCal = 0;
double speedLastTime = 0;
double deltaTime = 0.0;


double calSpeed = 0;
int counter = 0;

int analog_value;
double Angle;

uint8_t peerAddress[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };

#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
struct message {
  double value;
} __attribute__((packed));

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Init Pins
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);

  InitESP();

  xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
}

void SenderTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  message msg;

  while (1) {
    if(millis() - speedLastTime > 300) {
      calSpeed = 0;
    }
    msg.value = calSpeed;
    esp_now_send(peerAddress, (uint8_t*)&msg, sizeof(msg));
    Serial.print("Package sent: ");
    Serial.println(calSpeed);
    /*
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();*//*
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("Speed: ");
    Serial.println(calSpeed);*/
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(25));
  }
}

void loop() {
  // put your main code here, to run repeatedly:  
  
}

void Countup() {
  if (millis() - lasttimeDB >= delaytimer) {
    deltaTime = millis() - speedLastTime;
    speedLastTime = millis();
    //delaytimer = deltaTime * 0.35;
    calSpeed = 0.036 / (deltaTime / 1000.0);
    counter++;
    /*
    if(calSpeed < 0.18) {
      delaytimer = 100;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    else if(calSpeed < 0.28) {
      delaytimer = 90;
    }
    */

    lasttimeDB = millis();
  }
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
  message *msg = (message *)incomingData;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
}

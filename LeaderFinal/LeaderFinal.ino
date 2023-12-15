#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Diffie.h"

// ############# PINS #############
const int ServoPin = 32;  // GPIO 4 on the ESP  //A1 pin på Arduino
float input_voltage = 0.0;

const int OpticPin = 39;  //pin 3 på arduino

double lasttimeDB = 0;
double delaytimer = 60;

double speedTimeCal = 0;
double speedLastTime = 0;
double deltaTime = 0.0;

double angle = 0;
double calSpeed = 0;

double starttime = 0;
double exectime = 0;

TaskHandle_t SenderTaskHandle = NULL;

//08:3A:F2:45:44:BC
uint8_t peerAddress[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };
// uint8_t peerAddress[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };
static SemaphoreHandle_t keyMutex;
#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Init Pins
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
  keyMutex = xSemaphoreCreateMutex();

  InitESP();

  xTaskCreate(keyTask, "keyTask", 3000, NULL, 2, NULL);
  xTaskCreate(AngleCal, "AngleCal", 3000, NULL, 3, NULL);
  xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
}

void AngleCal(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  int analogValue = 0;
  double inputVoltage = 0;

  while (1) {
    analogValue = analogRead(ServoPin);
    inputVoltage = (analogValue * 4.0) / 4095.0;
    angle = 39.8613 * inputVoltage - 57.6003;
    //Serial.println(angle);
    
    vTaskDelayUntil(&lastWakeTime, 25);
  }
}


void keyTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    if (keyEstablished == false && myKeySent == false) {
      Serial.println(keyEstablished);
      Serial.println(myKeySent);
      Serial.println();
      getSharedKey();
    }


  }
}


void SenderTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    
    if (keyEstablished == true) {
      message msg;

      if (millis() - speedLastTime > 300) {
        calSpeed = 0;
      }

      msg.Velocity = calSpeed;
      msg.Angle = angle;

      msg.checksum = crc32((uint8_t *)&msg, sizeof(message) - sizeof(msg.checksum));

      encryptmsg((uint8_t *)&msg, sizeof(message), SharedSecret);

      esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(msg));
      Serial.print("Package sent: ");
      Serial.print(calSpeed);
      Serial.print("  |  ");
      Serial.println(angle);
    }
    vTaskDelayUntil(&lastWakeTime, 25);
  }
}


void loop() {
  // put your main code here, to run repeatedly:
}

void Countup() {
  if (millis() - lasttimeDB >= delaytimer) {
    deltaTime = millis() - speedLastTime;
    speedLastTime = millis();
    calSpeed = 0.036 / (deltaTime / 1000.0);
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
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("OnDataRecv");

  if (len == sizeof(Keymsg) && myKeySent == true) {
    Keymsg *kmsg = (Keymsg *)incomingData;
    if (kmsg->checksum == crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
      Serial.println("Received public key.");
      int skey = kmsg->PublicKey;
      SharedSecret = SecretKey(skey);

      Serial.println("Verified key public key.");
      Serial.println(SharedSecret);
      Serial.println("Key exchange complete.");
      keyEstablished = true;
      AckMsg ack;
      ack.ranack = 1;
      ack.checksum = crc32(&ack, sizeof(AckMsg) - sizeof(ack.checksum));

      esp_now_send(peerAddress, (uint8_t *)&ack, sizeof(AckMsg));

    } else {
      Serial.println("Checksum verification failed for received public key.");
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println("OnDataSent");
  if (status == ESP_NOW_SEND_SUCCESS) {
    //Serial.println("Sent with success");
  } else {
    resetKeyExchange();
  }
}

//===========================Diffie Hellman===========================//
void getSharedKey() {
  if (PrivateKey == 0) {
    PrivateKey = esp_random() % Prime;
  }
  Keymsg kmsg;
  kmsg.PublicKey = PublicKey();
  kmsg.checksum = crc32(&kmsg, sizeof(Keymsg) - sizeof(kmsg.checksum));
  myKeySent = true;
  esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
}

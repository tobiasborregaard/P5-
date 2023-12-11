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


double calSpeed = 0;
int counter = 0;

int analog_value;
double Angle;
TaskHandle_t SenderTaskHandle = NULL;

//08:3A:F2:45:44:BC
uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x44, 0xBC };
// uint8_t peerAddress[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };
static SemaphoreHandle_t keyMutex;
#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
bool keysExhanged = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Init Pins
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
  keyMutex=xSemaphoreCreateMutex();
  InitESP();
  xTaskCreate(keyTask, "keyTask", 3000, NULL, 1, NULL);
  
}


void keyTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  int hertz = 100;
  while (1) {
    if (keyEstablished == false && myKeySent == false) {
      getSharedKey();
    }
    if (keyEstablished == true) {
      hertz = hertzToMilliseconds(1);
    } 
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(hertz));
  }
}


void SenderTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  int hertz = 10;

  while (1) {
    Serial.println("1");
    if (keyEstablished == true ) {
      message msg;

      hertz = hertzToMilliseconds(55);
      if (millis() - speedLastTime > 300) {
        calSpeed = 0;
      }

      // msg.Velocity = calSpeed;
      // msg.Angle = analogRead(ServoPin);

      msg.Angle = esp_random() % 180;
      msg.Velocity = esp_random() % 100;

      msg.checksum = crc32((uint8_t *)&msg, sizeof(message) - sizeof(msg.checksum));

      encryptmsg((uint8_t *)&msg, sizeof(message), SharedSecret);

      esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(msg));
      Serial.print("Package sent: ");
      Serial.println(calSpeed);
      Serial.println("3");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(hertz));
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
      xSemaphoreTake(keyMutex, 100);
      keysExhanged = true;
      keyEstablished = true;
      xSemaphoreGive(keyMutex);
      AckMsg ack;
      ack.ranack = 1;
      ack.checksum = crc32(&ack, sizeof(AckMsg) - sizeof(ack.checksum));
      
      esp_now_send(peerAddress, (uint8_t *)&ack, sizeof(AckMsg));
      if(SenderTaskHandle == NULL) {
        xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, &SenderTaskHandle);
      }
    } else {
      Serial.println("Checksum verification failed for received public key.");
    }
  }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println("OnDataSent");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sent with success");
  } else {
    myKeySent = false;
  }
}



//===========================ESP NOW===========================//





//===========================Diffie Hellman===========================//
void getSharedKey() {
  if (myKeySent == false) {
    if (PrivateKey == 0) {
      PrivateKey = esp_random() % Prime;
    }
    Keymsg kmsg;
    kmsg.PublicKey = PublicKey();
    kmsg.checksum = crc32(&kmsg, sizeof(Keymsg) - sizeof(kmsg.checksum));
    xSemaphoreTake(keyMutex, 100);
    myKeySent = true;
    xSemaphoreGive(keyMutex);
    esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
  }
}

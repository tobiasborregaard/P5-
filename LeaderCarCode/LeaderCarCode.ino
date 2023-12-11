#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Diifie.h"


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

//===========Retransmit==============//
double PrevAngle;
double PrevVelocity;
int unknownCounter = 0;
uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x44, 0xBC };
// uint8_t peerAddress[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };

#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;

//=====sending stuff=====//

TaskHandle_t keyexchangeHandle = NULL;
TaskHandle_t SenderTaskhandle = NULL;


DeviceState deviceState = IDLE;
bool checkOnce = false;

unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;
int HZ = 0;
int hertzToMilliseconds(int hertz) {
  if (hertz <= 0) {
    Serial.println("Hertz must be greater than zero.");
    return 1;  // Return a default value or handle the error as needed
  }
  double result = 1000 / hertz;
  Serial.print("HZ to ms: ");
  Serial.println(result);
  return static_cast<int>(result);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Init Pins
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
  HZ = hertzToMilliseconds(60);
  InitESP();

  xTaskCreate(KeyExchangeTask, "KeyExchangeTask", 3000, NULL, 1, &keyexchangeHandle);

  // xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
}

void SenderTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  message msg;

  while (1) {
    if(millis() - speedLastTime > 300) {
      calSpeed = 0;
    }
    msg.Velocity = calSpeed;
    PrevVelocity = calSpeed;
    msg.Angle = analogRead(ServoPin);
    PrevAngle = analogRead(ServoPin);

    esp_now_send(peerAddress, (uint8_t*)&msg, sizeof(msg));

    vTaskDelayUntil(&lastWakeTime, HZ);
  }
}

void statusTask(void *pvParameters) {

  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    if (deviceState == CONNECTED) {
      if (keyEstablished == true) {
        if (checkOnce == false) {

          xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
          checkOnce = true;
        } else {
          return;
        }
      }
    }
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

void KeyExchangeTask(void *pvParameters) {


  while (1) {

  unsigned long currentMillis = millis();

    if (deviceState == IDLE) {
      if (currentMillis - lastHelloTime >= helloInterval) {
        resetKeyExchange();  // Reset the key exchange state
        checkOnce = false;

        if (SenderTaskhandle != NULL) {
          vTaskDelete(SenderTaskhandle);

        }

        lastHelloTime = currentMillis;
        // Send "hello" message and transition to WAITING_FOR_ACK
        HelloMsg msg;
        msg.ranhello = random(0, 255);
        msg.checksum = crc32((uint8_t *)&msg, sizeof(HelloMsg) - sizeof(msg.checksum));
        esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(HelloMsg));
        deviceState = WAITING_FOR_ACK;

        lastAckTime = currentMillis;  // Start the timeout for the acknowledgement
      }
    }
    if (deviceState == WAITING_FOR_ACK) {
      if (currentMillis - lastAckTime >= ackTimeout) {

        Serial.println("ACK timed out.");
        deviceState = IDLE;
      }
    }
    if (deviceState == KEY_EXCHANGE) {
      if (myKeySent == false) {
        Serial.println("KEY_EXCHANGE");
        Keymsg kmsg;
        int key = PublicKey();
        kmsg.PublicKey = key;
        kmsg.checksum = crc32((uint8_t *)&kmsg, sizeof(Keymsg) - sizeof(kmsg.checksum));
        esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
        myKeySent = true;
        lastKeyExchangeTime = currentMillis;  // Start the timeout for the key exchange
      } else if (currentMillis - lastKeyExchangeTime >= keyExchangeTimeout) {
        // If timeout, go back to IDLE
        Serial.println("Key exchange timed out.");
        deviceState = IDLE;
      }
    }

    if (deviceState == WAITING_KEY_EXCHANGE) {
      return;
    }
    if (deviceState == CONNECTED) {
      xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
    }
  }
}
//=============ESP-NOW=================//
// Handle received data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Handle incoming data based on the current state
  unknownCounter++;

  if (unknownCounter > 10) {
    deviceState = IDLE;
    // Send back an "acknowledgement" message
    AckMsg msg;
    msg.ranack = 80113;
    msg.checksum = crc32((uint8_t *)&msg, sizeof(AckMsg) - sizeof(msg.checksum));
    esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(msg));
  } else {
    switch (deviceState) {
      case IDLE:
        // Handle "hello" message
        if (len == sizeof(HelloMsg)) {
          unknownCounter = 0;
          HelloMsg *hmsg = (HelloMsg *)incomingData;
          Serial.println("Received hello message");
          if (hmsg->checksum == crc32(hmsg, sizeof(HelloMsg) - sizeof(hmsg->checksum))) {

            // Send back an "acknowledgement" message
            AckMsg msg;
            msg.ranack = 31108;
            msg.checksum = crc32((uint8_t *)&msg, sizeof(AckMsg) - sizeof(msg.checksum));

            deviceState = WAITING_KEY_EXCHANGE;
            esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(msg));
          } else {
            Serial.println("Checksum verification failed for received hello message.");
          }
        }
        break;

      case WAITING_FOR_ACK:
        // Handle acknowledgement
        Serial.println("Received ack message");
        if (len == sizeof(AckMsg)) {
          unknownCounter = 0;
          AckMsg *amsg = (AckMsg *)incomingData;
          if (amsg->checksum == crc32(amsg, sizeof(AckMsg) - sizeof(amsg->checksum))) {
            Serial.println("ACK verified");
            deviceState = KEY_EXCHANGE;
          } else {
            Serial.println("Checksum verification failed for received ack message.");
          }
        } else {
          Serial.println("Wrong size for ack message.");
        }

        break;
      case KEY_EXCHANGE:
        handleKeyExchange(incomingData, len, peerAddress, myKeySent, deviceState, "Key_exchange");
        unknownCounter = 0;
        if (deviceState != CONNECTED and keyEstablished == true) {
          deviceState = CONNECTED;
        }
        break;
      case WAITING_KEY_EXCHANGE:
        handleKeyExchange(incomingData, len, peerAddress, myKeySent, deviceState, "waiting_Key_exchange");
        unknownCounter = 0;
        if (deviceState != CONNECTED and keyEstablished == true) {
          deviceState = CONNECTED;
        }
        break;

      case CONNECTED:
        if (len == sizeof(retmsg)) {
          unknownCounter = 0;
          retmsg *rmsg = (retmsg *)incomingData;
          Serial.println("Received ret message");
          message msg;
          msg.Angle = PrevAngle;
          msg.Velocity = PrevVelocity;
          encryptmsg((uint8_t *)&msg, sizeof(message), SharedSecret);
          addChecksumToMessage(&msg);

          esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(message));
        }
   
    }
    //====only check for error=====//
    if (len == sizeof(AckMsg) && deviceState != WAITING_FOR_ACK) {
      unknownCounter = 0;
      AckMsg *amsg = (AckMsg *)incomingData;
      if (amsg->checksum == crc32(amsg, sizeof(AckMsg) - sizeof(amsg->checksum))) {
        if (amsg->ranack == 80113) {
          deviceState = IDLE;
          if (keyexchangeHandle != NULL) {
            xTaskCreate(KeyExchangeTask, "KeyExchangeTask", 3000, NULL, 1, &keyexchangeHandle);
          }
        }
      } else {
        Serial.println("Checksum verification failed for received ack message.");
      }
    } else {
      Serial.println("Wrong size for ack message.");
    }
  }
}


// Handle sending data failure
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    if (deviceState == CONNECTED) {
      
      deviceState = IDLE;
    }
    Serial.println("Failed to send data");
  }
}

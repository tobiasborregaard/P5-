#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x44, 0xBC };

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
  InitESP();

  xTaskCreate(SenderTask, "SenderTask", 3000, NULL, 1, NULL);
}

void SenderTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  message msg;

  while (1) {
    msg.value = 10.0;
    esp_now_send(peerAddress, (uint8_t*)&msg, sizeof(msg));
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
  }
}

void loop() {
  // put your main code here, to run repeatedly:  
  
}

// ESP-NOW
void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();
  //Serial.println(WiFi.macAddress());
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
  Serial.print("Message received: ");
  Serial.println(msg->value);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
}

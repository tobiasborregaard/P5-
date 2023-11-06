#include <esp_now.h>
#include <WiFi.h>



// Add the MAC addresses of the receivers



// Replace with the MAC address of your receiver
const uint8_t receiverMAC[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };

// Structure example to send data
typedef struct struct_message {
  char a[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
  } else {
    Serial.println("Delivery Fail");
  }
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    esp_now_init();
    Serial.println("ESP-NOW Initialized");
  }



  // Get MAC Address
  Serial.println(WiFi.macAddress());

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Peer interface information
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  strcpy(myData.a, "Hello from sender!");

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(2000);
}

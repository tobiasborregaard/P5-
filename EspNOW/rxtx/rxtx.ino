#include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>

// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();  // Corrected: added parentheses
BluetoothSerial SerialBT;

#define Width 135
#define Height 240

//ttgo
uint8_t peerAddress[] = {0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8};
// lilygo
//uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };


#define CHANNEL 0

esp_now_peer_info_t peerInfo;
// skal ændres så vi kan sende vores egen pakke struktur
struct message {
  char text[32];
  int value;
} __attribute__((packed));  // Ensure no padding is added to the struct

//hvis skærmen skal bruges til noget
void tftsetup(){
tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor((Width / 2)-38, Height / 2);
  tft.print(WiFi.macAddress());


}
// kan sende data til en computer med bluetooth
void bluetoothsetup(){
    SerialBT.begin("Der fuhrer"); //Bluetooth for leading car
    // Serial.



}
// esp now initialisering
void InitESP() {
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();  // Ensure we're not connected to any WiFi network (optional)

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW initialization successful");
  } else {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = CHANNEL;  // Set the peer channel
  peerInfo.encrypt = false;    // Data will not be encrypted

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
}

void setup() {
  Serial.begin(115200);
  tftsetup();
  bluetoothsetup();
  InitESP(); 



}


void loop() {
  // Nothing to do here
}

// Callback when data is received
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int data_len) {
  if (data_len == sizeof(message)) {
    const message* msg = reinterpret_cast<const message*>(incomingData);
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; ++i) {
      if (mac_addr[i] < 16) {
        Serial.print("0"); // Print a leading zero if necessary
      }
      Serial.print(mac_addr[i], HEX);
      if (i < 5) {
        Serial.print(":"); // Print a colon after each byte except the last
      }
    }
    Serial.println(); // Print a newline after the MAC address

    Serial.print("Message: ");
    Serial.println(msg->text);
  } else {
    Serial.println("Received data size does not match expected message size.");
  }
}


void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendData(const message* dataToSend) {
  esp_err_t result = esp_now_send(peerAddress, (uint8_t*)dataToSend, sizeof(message));

  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}
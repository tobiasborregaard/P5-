#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>


//ttgo
//uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 };
// lilygo
uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };

esp_now_peer_info_t peerInfo;
// skal ændres så vi kan sende vores egen pakke struktur
struct message {
  float Angle;
  float Speed;
} __attribute__((packed));  // Ensure no padding is added to the struct

// Initialize TFT display
TFT_eSPI tft = TFT_eSPI(); 

#define Width 135
#define Height 240

#define CHANNEL 0
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
}

void tftsetup() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 80);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Text color, background color
  tft.print(WiFi.macAddress());
  tft.setCursor(0,20);
  tft.print("Angle: 0 , Speed: 0");
}
void setup() {
  Serial.begin(115200);
  InitESP();
  tftsetup();

}

void loop() {
  // put your main code here, to run repeatedly:

}
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
  if (len == sizeof(message)) {
    const message* msg = reinterpret_cast<const message*>(incomingData);
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; ++i) {
      if (mac_addr[i] < 16) {
        Serial.print("0");  // Print a leading zero if necessary
      }
      Serial.print(mac_addr[i], HEX);
      if (i < 5) {
        Serial.print(":");  // Print a colon after each byte except the last
      }
    }
    Serial.println();  
    String package ="Angle: " + String(msg->Angle) + " , Speed: " + String(msg->Speed);
    Serial.println(package);
    tft.setCursor(0,20);
    tft.print(package);
    

    
  } else {
    Serial.println("Received data size does not match expected message size.");
  }
}
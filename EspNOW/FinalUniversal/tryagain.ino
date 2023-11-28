#include <esp_now.h>
#include <WiFi.h>
#include <cmath>


uint8_t peerAddress1[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 }; // Updated to match TTGO: 3C:61:05:0B:BB:90
uint8_t peerAddress2[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 }; // Matches with LILYGO: 08:3A:F2:69:CF:64
#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
struct message {
  double Angle;
  float Speed;
  uint8_t checksum;
} __attribute__((packed));

struct Keymsg {
  int PublicKey;
  uint8_t checksum;
} __attribute__((packed));

struct HelloMsg {
  uint8_t ranhello;
  uint8_t checksum;
} __attribute__((packed));

struct AckMsg {
  uint16_t ranack;
  uint8_t checksum;
} __attribute__((packed));


enum State { IDLE,KEY_NOTSENT,KEY_ESTABLISHED, KEY_SENT, KEY_RECEIVED,HELLO_RECEIVED,HELLO_SENT,ACK_RECEIVED };



bool keyEstablished = false;  // Flag to check if the key has been established
bool isInitiator = false;


// Constants for Diffie-Hellman key exchange
const int Prime = 707898413;
const int Generator = 2;
int PublicKey = 0;
int PrivateKey = 0;
int SharedSecret = 0;

int KEY_EXCHANGE_TIMEOUT = 1000;



void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();    
  Serial.println(WiFi.macAddress());    
  // Initialize ESP-NOW, restart on failure
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }

  // Set up peer information with address and channel
  uint8_t peerAddress[6];
  if (WiFi.macAddress() == String("08:3A:F2:45:3D:E8")) {
  memcpy(peerAddress, peerAddress2, sizeof(peerAddress1));
} else {
  memcpy(peerAddress, peerAddress1, sizeof(peerAddress2));
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

void setup() {
  Serial.begin(115200);  // Initialize Serial communication
  InitESP();             // Call initialization function for ESP-NOW
}

void loop() {


}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){




}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println("Data sent");
}

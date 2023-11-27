#include <esp_now.h>
#include <WiFi.h>
#include <cmath>

// Constants for Diffie-Hellman key exchange
const int Prime = 707898413;
const int Generator = 2;

// Variables for public and private keys, and the shared secret
int PublicKey = 0;
int PrivateKey = 0;
int SharedSecret = 0;
#define CHANNEL 0
bool keyEstablished = false;  // Flag to check if the key has been established

// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;

struct message {
  double Angle;
  float Speed;
  uint8_t checksum;
} __attribute__((packed));

struct Keymsg {
  int PublicKey;
  uint8_t checksum;
} __attribute__((packed));
// ESP32 MAC addresses for ttgo and lilygo
//ttgo
// uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 };
// lilygo
uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };


// Function prototypes
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void establishKey();
void encryptmsg(uint8_t *message, size_t messageSize);
void decryptmsg(uint8_t *message, size_t messageSize);
void addChecksumToMessage(message *msg);
void printMessageBytes(message msg);
uint8_t calculateChecksum(const void *data, size_t length);

// Initialize ESP-NOW with basic settings
void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();    // Disconnect from any Wi-Fi network

  // Initialize ESP-NOW, restart on failure
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }

  // Set up peer information with address and channel
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

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(Keymsg)) {
    Keymsg *kmsg = (Keymsg *)incomingData;
    // Verify checksum
    if (kmsg->checksum == calculateChecksum(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
      // Calculate the shared secret
      SharedSecret = modularExponentiation(kmsg->PublicKey, PrivateKey, Prime);
      keyEstablished = true;
      syncKey();
      if (ESP_NOW_SEND_SUCCESS == "Succes"){
      Serial.println("Key exchange complete. Shared secret established.");

      }else{
        Serial.println("Sending failed")
      }
    } else {
      Serial.println("Checksum verification failed for received public key.");
    }
  } else if (len == sizeof(message)) {
    message *msg = (message *)incomingData;
    decryptmsg((uint8_t *)msg, sizeof(message));
    Serial.print("Received: ");
    printMessageBytes(*msg);
    Serial.println();
  }
}

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Function to encrypt a message
void encryptmsg(uint8_t *message, size_t messageSize) {
  for (size_t i = 0; i < messageSize; i++) {
    message[i] = message[i] ^ (SharedSecret >> (8 * (i % 4)));  // XOR encryption
  }

  // Print the encrypted message in a human-readable format
  for (size_t i = 0; i < messageSize; i++) {
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// Function to decrypt a message
void decryptmsg(uint8_t *message, size_t messageSize) {
  for (size_t i = 0; i < messageSize; i++) {
    message[i] = message[i] ^ (SharedSecret >> (8 * (i % 4)));  // XOR decryption
  }
}

// Function to add a checksum to a message
void addChecksumToMessage(message *msg) {
  msg->checksum = calculateChecksum(msg, sizeof(message) - sizeof(msg->checksum));
}

// Function to calculate checksum of a message
uint8_t calculateChecksum(const void *data, size_t length) {
  uint8_t checksum = 0;
  const uint8_t *bytes = (const uint8_t *)data;

  for (size_t i = 0; i < length; ++i) {
    checksum += bytes[i];  // Sum the bytes for checksum
  }

  return checksum;
}
int modularExponentiation(int base, int exponent, int modulus) {
  long long result = 1;
  long long x = base % modulus;

  while (exponent > 0) {
    if (exponent % 2 == 1) {
      result = (result * x) % modulus;
    }
    x = (x * x) % modulus;
    exponent >>= 1;
  }

  return static_cast<int>(result);
}

// Function to synchronize the key using Diffie-Hellman method
void syncKey() {
  //generate once
  if (PrivateKey == 0) {
    PrivateKey = esp_random();
    PublicKey = modularExponentiation(Generator, PrivateKey, Prime);
  }

  Keymsg kmsg;
  kmsg.PublicKey = PublicKey;                                                      // Set the public key in the message
  kmsg.checksum = calculateChecksum(&kmsg, sizeof(kmsg) - sizeof(kmsg.checksum));  // Calculate the checksum

  // Send the public key message
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(kmsg));
  if (result == ESP_OK) {
    Serial.println("Public key sent");
  } else {
    Serial.println("Error sending public key");
  }
}

// Function to print message bytes
void printMessageBytes(message msg) {
  Serial.print("Angle: ");
  Serial.print(msg.Angle);
  Serial.print(", Speed: ");
  Serial.print(msg.Speed);
  Serial.print(", Checksum: ");
  Serial.print(msg.checksum);
}

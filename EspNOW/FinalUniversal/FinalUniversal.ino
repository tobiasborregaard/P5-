 #include <esp_now.h>
#include <WiFi.h>
#include <cmath>

// Constants for Diffie-Hellman key exchange
const int Prime = 707898413;
const int Generator = 2;
enum State { IDLE,KEY_NOTSENT,KEY_ESTABLISHED, KEY_SENT, KEY_RECEIVED,HELLO_RECEIVED,HELLO_SENT,ACK_RECEIVED };
State deviceState = IDLE;
// Variables for public and private keys, and the shared secret
int PublicKey = 0;
int PrivateKey = 0;
int SharedSecret = 0;
#define CHANNEL 0
bool keyEstablished = false;  // Flag to check if the key has been established
bool isInitiator = false;


int KEY_EXCHANGE_TIMEOUT = 1000;



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

struct HelloMsg {
  uint8_t ranhello;
  uint8_t checksum;
} __attribute__((packed));

struct AckMsg {
  uint16_t ranack;
  uint8_t checksum;
} __attribute__((packed));



uint8_t peerAddress1[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 }; // Updated to match TTGO: 3C:61:05:0B:BB:90
uint8_t peerAddress2[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 }; // Matches with LILYGO: 08:3A:F2:69:CF:64

// Function prototypes
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void establishKey();
void encryptmsg(uint8_t *message, size_t messageSize);
void decryptmsg(uint8_t *message, size_t messageSize);
void addChecksumToMessage(message *msg);
void printMessageBytes(message msg);
uint8_t calculateChecksum(const void *data, size_t length);
uint8_t peerAddress[]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Initialize ESP-NOW with basic settings
void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();    // Disconnect from any Wi-Fi network
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
  // Continuously attempt to sync key if not established
  if (!keyEstablished) {
    establishKey();
    }
}

// Function to establish the key
void establishKey() {
  Serial.println("Starting key establishment...");
    deviceState = IDLE;
  isInitiator = false;
  // Wait a random amount of time before retrying
  delay(random(200, 1000)); // Wait between 0,2 and 1 seconds
  syncKey();

} 



void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("Data received");

  if (len == sizeof(HelloMsg) && deviceState == IDLE) {
    HelloMsg* hmsg = (HelloMsg*)incomingData;
    if (hmsg->checksum == calculateChecksum(hmsg, sizeof(HelloMsg) - sizeof(hmsg->checksum))) {
      deviceState = HELLO_RECEIVED;
      if (!isInitiator) {
        // Send back an "acknowledgement" message
        AckMsg amsg;
        amsg.checksum = calculateChecksum(&amsg, sizeof(amsg) - sizeof(amsg.checksum));
        esp_now_send(peerAddress, (uint8_t *)&amsg, sizeof(amsg));
      }
    } else {
      Serial.println("Checksum verification failed for received hello message.");
    }
  }

  if (len == sizeof(AckMsg) && deviceState == HELLO_RECEIVED && isInitiator) {
    AckMsg* amsg = (AckMsg*)incomingData;
    if (amsg->checksum == calculateChecksum(amsg, sizeof(AckMsg) - sizeof(amsg->checksum))) {
      deviceState = ACK_RECEIVED;
    } else {
      Serial.println("Checksum verification failed for received acknowledgement message.");
    }
  }

  if (len == sizeof(Keymsg) && deviceState == ACK_RECEIVED) {
    Keymsg *kmsg = (Keymsg *)incomingData;
    // Verify checksum
    if (kmsg->checksum == calculateChecksum(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
      // Calculate the shared secret
      SharedSecret = modularExponentiation(kmsg->PublicKey, PrivateKey, Prime);
      keyEstablished = true;
      deviceState = KEY_ESTABLISHED;
      Serial.println("Key exchange complete. Shared secret established.");
    } else {
      Serial.println("Checksum verification failed for received public key.");
    }
  } else if (len == sizeof(message) && deviceState == KEY_ESTABLISHED) {
    message *msg = (message *)incomingData;
    decryptmsg((uint8_t *)msg, sizeof(message));
    Serial.print("Received: ");
    printMessageBytes(*msg);
    Serial.println();
  }
}






// Function to handle the status of sent data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println("Data sent");
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Failed to send data");
  } else {
    Serial.println("Data sent successfully");
  }
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





//exponentiation by squaring method
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







void syncKey() {
  Serial.println("syncKey called");

  // Delay to avoid rapid retransmissions
  delay(random(100, 5000));
  Serial.println("After random delay");

  if (deviceState == IDLE) {
    Serial.println("Device State: IDLE");
    if (!isInitiator) {
      Serial.println("Not an initiator, sending HelloMsg");
      HelloMsg hmsg;
      hmsg.checksum = calculateChecksum(&hmsg, sizeof(hmsg) - sizeof(hmsg.checksum));
      esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&hmsg, sizeof(hmsg));
      if (result == ESP_OK) {
        Serial.println("HelloMsg sent successfully");
      } else {
        Serial.println("Error sending HelloMsg");
      }
      deviceState = HELLO_SENT;
      isInitiator = true;
    } else {
      Serial.println("Already an initiator, not sending HelloMsg again");
    }
  } else if (deviceState == HELLO_RECEIVED) {
    Serial.println("Device State: HELLO_RECEIVED, proceeding with key exchange");
    if (PrivateKey == 0) {
      PrivateKey = esp_random();
      PublicKey = modularExponentiation(Generator, PrivateKey, Prime);
      Serial.print("Generated PrivateKey: "); Serial.println(PrivateKey);
      Serial.print("Generated PublicKey: "); Serial.println(PublicKey);
    }
    Keymsg kmsg;
    kmsg.PublicKey = PublicKey;
    kmsg.checksum = calculateChecksum(&kmsg, sizeof(kmsg) - sizeof(kmsg.checksum));
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(kmsg));
    if (result == ESP_OK) {
      deviceState = KEY_SENT;
      Serial.println("PublicKey sent successfully");
    } else {
      Serial.println("Error sending public key");
    }
  } else {
    Serial.print("Device State: "); Serial.println(deviceState);
    Serial.println("No action taken");
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

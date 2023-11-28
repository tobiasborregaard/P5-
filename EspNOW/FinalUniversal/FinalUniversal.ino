#include <esp_now.h>
#include <WiFi.h>



uint8_t peerAddress1[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 };  // Updated to match TTGO: 3C:61:05:0B:BB:90
uint8_t peerAddress2[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };  // Matches with LILYGO: 08:3A:F2:69:CF:64
uint8_t *peerAddress;
#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
struct message {
  double Angle;
  float Velocity;
  uint32_t checksum;
};

struct Keymsg {
  int PublicKey;
  uint32_t checksum;
};

struct HelloMsg {
  uint8_t ranhello;
  uint32_t checksum;
};

struct AckMsg {
  uint16_t ranack;
  uint32_t checksum;
};


enum DeviceState {
  IDLE,
  WAITING_FOR_ACK,
  KEY_EXCHANGE,
  CONNECTED,
  WAITING_KEY_EXCHANGE
};

enum LeaderState {
  LEADER,
  FOLLOWER
};

DeviceState deviceState = IDLE;


bool myKeySent = false;
bool keyEstablished = false;  // Flag to check if the key has been established
bool isInitiator = false;


// Constants for Diffie-Hellman key exchange
const int Prime = 707898413;
const int Generator = 2;

int PrivateKey = 0;
int SharedSecret = 0;


long KEY_EXCHANGE_TIMEOUT = 800;



void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  // Initialize ESP-NOW, restart on failure
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }

  // Set up peer information with address and channel
  if (WiFi.macAddress() == String("08:3A:F2:45:3D:E8")) {
    peerAddress = peerAddress2;
  } else {
    peerAddress = peerAddress1;
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
  switch (deviceState) {
    case IDLE:
      {
        myKeySent = false;
        Serial.println("IDLE");
        // Send "hello" message and transition to WAITING_FOR_ACK
        delay(random(300, 500));  // Wait between 0,2 and 0,5 seconds
        HelloMsg msg;
        msg.ranhello = random(0, 255);
        msg.checksum = crc32((uint8_t *)&msg, sizeof(HelloMsg) - sizeof(msg.checksum));
        esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(HelloMsg));
        deviceState = WAITING_FOR_ACK;
        break;
        case WAITING_FOR_ACK:
          // If timeout, resend "hello" message
          long timmil = millis();
          while (millis() - timmil < KEY_EXCHANGE_TIMEOUT) {
            if (deviceState == KEY_EXCHANGE) {
              break;
            }
          }
          if (deviceState == WAITING_FOR_ACK) {
            Serial.println("ACK timed out.");
            deviceState = IDLE;
          }

          break;
      }
    case KEY_EXCHANGE:
      {
        Serial.println("KEY_EXCHANGE");
        delay(20);
        // If key exchange complete, transition to CONNECTED
        if (myKeySent == false) {
          Keymsg kmsg;
          kmsg.PublicKey = PublicKey();
          kmsg.checksum = crc32((uint8_t *)&kmsg, sizeof(Keymsg) - sizeof(kmsg.checksum));
          esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
          myKeySent = true;
        }
        long timmil = millis();


        while (millis() - timmil < KEY_EXCHANGE_TIMEOUT) {
          if (deviceState == KEY_EXCHANGE) {
            break;
          }
        }
        if (deviceState == KEY_EXCHANGE) {
          
          Serial.println("Key exchange timed out.");
          deviceState = IDLE;
        }


        break;
      }
    case WAITING_KEY_EXCHANGE:
      {
        if(isInitiator == false){
          Serial.println("WAITING_KEY_EXCHANGE");
          delay(20);
          isInitiator = true; 
          
        
      }
      }
    case CONNECTED:
      {
        
       
        break;
      }
  }
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Handle incoming data based on the current state
  Serial.println("Data received");
  switch (deviceState) {
    case IDLE:
      // Handle "hello" message
      if (len == sizeof(HelloMsg)) {
        HelloMsg *hmsg = (HelloMsg *)incomingData;
        Serial.println("Received hello message");
        if (hmsg->checksum == crc32(hmsg, sizeof(HelloMsg) - sizeof(hmsg->checksum))) {
          delay(20);
          // Send back an "acknowledgement" message
         
          AckMsg msg;

          msg.ranack = 31108;
          msg.checksum = crc32((uint8_t *)&msg, sizeof(AckMsg) - sizeof(msg.checksum));
          
          esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(msg));
          delay(20);
          deviceState = WAITING_KEY_EXCHANGE;
        } else {
          Serial.println("Checksum verification failed for received hello message.");
        }
      }
      break;

    case WAITING_FOR_ACK:
      // Handle acknowledgement
      Serial.println("Received ack message");
      

      if (len == sizeof(AckMsg)) {

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
      // Handle key exchange
      if (len == sizeof(Keymsg) ) {
        Serial.println("Received public key.");
        Keymsg *kmsg = (Keymsg *)incomingData;
        if (kmsg->checksum == crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
          SharedSecret = SecretKey(kmsg->PublicKey);

          Serial.println("Verified key public key.");
          if (myKeySent == false) {
            kmsg->PublicKey = PublicKey();
            kmsg->checksum = crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum));
            esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
            Serial.println("Sent public key.");
             // Send message
              Serial.println("Works nicely");
            deviceState = CONNECTED;
            myKeySent = true;
          } else {
            // Send message
              Serial.println("Works nicely");
            deviceState = CONNECTED;
          }

        } else {

          Serial.println("Checksum verification failed for received public key.");
        }
      } else
      {
        Serial.println("Wrong size for key message.");
      }
      
      break;
    case WAITING_KEY_EXCHANGE:
      // Handle key exchange
      
      if (len == sizeof(Keymsg)) {
          Serial.println("Received public key.");
        Keymsg *kmsg = (Keymsg *)incomingData;
        if (kmsg->checksum == crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
          SharedSecret = SecretKey(kmsg->PublicKey);
          Serial.println("Verified key waitn public key.");
          if (myKeySent == false) {
            kmsg->PublicKey = PublicKey();
            kmsg->checksum = crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum));
            esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
            myKeySent = true;
            deviceState = CONNECTED;
          } else {
            deviceState = CONNECTED;
          }

        } else {
          Serial.println("Checksum verification failed for received public key.");
        }
      } else {
        Serial.println("Wrong size for key message.");
      }
      break;
    case CONNECTED:
      // Handle normal communication
      if (len == sizeof(message)) {
        message *msg = (message *)incomingData;
        decryptmsg((uint8_t *)msg, sizeof(message));
        Serial.print("Received: ");
        Serial.print(msg->Angle);
        Serial.print(" ");
        Serial.print(msg->Velocity);
        Serial.println();
      }
      break;
  }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Failed to send data");
  } 
}


// Function to encrypt a message
void encryptmsg(uint8_t *message, size_t messageSize) {
  for (size_t i = 0; i < messageSize; i++) {
    message[i] = message[i] ^ (SharedSecret >> (8 * (i % 4)));  // XOR encryption
  }
}

// Function to decrypt a message
void decryptmsg(uint8_t *message, size_t messageSize) {
  for (size_t i = 0; i < messageSize; i++) {
    message[i] = message[i] ^ (SharedSecret >> (8 * (i % 4)));  // XOR decryption
  }
}


// Function to add a checksum to a message
void addChecksumToMessage(message *msg) {
  msg->checksum = crc32(msg, sizeof(message) - sizeof(msg->checksum));
}



uint32_t crc32(const void *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes = static_cast<const uint8_t*>(data);

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint32_t)bytes[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;  // 0xEDB88320 is the reversed polynomial for CRC-32
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;  // Final negation
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

// publickey generate function
int PublicKey() {
  if (PrivateKey == 0) {
    PrivateKey = esp_random() % Prime;
  }


  return modularExponentiation(Generator, PrivateKey, Prime);
}

// secretkey generate function
int SecretKey(int rPublicKey) {
  return modularExponentiation(rPublicKey, PrivateKey, Prime);
}
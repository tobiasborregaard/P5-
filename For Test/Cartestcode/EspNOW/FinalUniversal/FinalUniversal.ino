#include <esp_now.h>
#include <WiFi.h>

uint8_t peerAddress1[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 };  // Updated to match TTGO: 3C:61:05:0B:BB:90
uint8_t peerAddress2[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };  // Matches with LILYGO: 08:3A:F2:69:CF:64
uint8_t peerAddress3[] = { 0x94, 0xB5, 0x55, 0xF9, 0x06, 0x44 };  // Matches with esp32s: 94:B5:55:F9:06:44
uint8_t *peerAddress;
#define CHANNEL 0
// Structure for ESP-NOW peer information and message format
esp_now_peer_info_t peerInfo;
//message types
struct message {
  double Angle;
  double Velocity;
  uint32_t checksum;
} __attribute__((packed));

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

struct retmsg {
  bool ret;
};

enum DeviceState {
  IDLE,
  WAITING_FOR_ACK,
  KEY_EXCHANGE,
  CONNECTED,
  WAITING_KEY_EXCHANGE,

};

enum LeaderState {
  Undefined,
  LEADER,
  FOLLOWER
};

DeviceState deviceState = IDLE;
LeaderState leaderState = Undefined;

bool myKeySent = false;
bool keyEstablished = false;  // Flag to check if the key has been established
bool isInitiator = false;

// Constants for Diffie-Hellman key exchange
const int Prime = 707898413;
const int Generator = 2;

int PrivateKey = 0;
int SharedSecret = 0;

unsigned long lastHelloTime = 0;
unsigned long helloInterval = random(200, 4000);
unsigned long lastAckTime = 0;
unsigned long ackTimeout = 2000;
unsigned long lastKeyExchangeTime = 0;
unsigned long keyExchangeTimeout = 2000;
unsigned long lastSendTime = 0;
unsigned long sendInterval = 1000;
int maxhertz = 1000;

double PrevAngle;
float PrevVelocity;


// int hertzToMilliseconds(int hertz) {
//     if (hertz <= 0) {
//         Serial.println("Hertz must be greater than zero.");
//         return 0; // Return a default value or handle the error as needed
//     }
//     double result = 1000 / hertz;
//     Serial.print("HZ to ms: ");
//     Serial.println(result);
//     return static_cast<int>(result);
// }


int packetsPerSecondCounter = 0;

int lastSecondCounter = 0;

int lastSecondTime = 0;

int packetloss = 0;

// int dsHZ = 10;
// int HZ = hertzToMilliseconds(10);

void InitESP() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to Station
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  // Initialize ESP-NOW, restart on failure
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }

  //  if (WiFi.macAddress() == String("08:3A:F2:45:3D:E8")) {
  //     peerAddress = peerAddress1; // or peerAddress3
  // // }
  if (WiFi.macAddress() == String("08:3A:F2:69:CF:64")) {
    peerAddress = peerAddress3;  // or peerAddress3
  } else {
    peerAddress = peerAddress2;  // or peerAddress2
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
  unsigned long currentMillis = millis();
  switch (deviceState) {
    case IDLE:
      {
        isInitiator = false;
        myKeySent = false;
        keyEstablished = false;
        leaderState = Undefined;
        // dsHZ = 10;
        // HZ = hertzToMilliseconds(dsHZ);

        Serial.println("IDLE");

        // Send "hello" message and transition to WAITING_FOR_ACK

        if (currentMillis - lastHelloTime >= helloInterval) {

          lastHelloTime = currentMillis;
          // Send "hello" message and transition to WAITING_FOR_ACK
          HelloMsg msg;
          msg.ranhello = random(0, 255);
          msg.checksum = crc32((uint8_t *)&msg, sizeof(HelloMsg) - sizeof(msg.checksum));
          esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(HelloMsg));
          deviceState = WAITING_FOR_ACK;

          lastAckTime = currentMillis;  // Start the timeout for the acknowledgement
        }
        break;
      }
    case WAITING_FOR_ACK:
      {

        if (currentMillis - lastAckTime >= ackTimeout) {

          Serial.println("ACK timed out.");
          deviceState = IDLE;
        }

        break;
      }
    case KEY_EXCHANGE:
      {

        Serial.println("KEY_EXCHANGE");
        if (myKeySent == false) {
          Keymsg kmsg;
          int key = PublicKey();
          kmsg.PublicKey = key;
          // Serial.println("Sent public key: ");
          // Serial.println(kmsg.PublicKey);
          // Serial.println("Sent public key");
          kmsg.checksum = crc32((uint8_t *)&kmsg, sizeof(Keymsg) - sizeof(kmsg.checksum));
          esp_now_send(peerAddress, (uint8_t *)&kmsg, sizeof(Keymsg));
          myKeySent = true;
          lastKeyExchangeTime = currentMillis;  // Start the timeout for the key exchange
        } else if (currentMillis - lastKeyExchangeTime >= keyExchangeTimeout) {
          // If timeout, go back to IDLE
          Serial.println("Key exchange timed out.");
          deviceState = IDLE;
        }

        break;
      }
    case WAITING_KEY_EXCHANGE:
      {

        break;
      }


    case CONNECTED:
      {

        if (leaderState == LEADER) {
          // if (currentMillis - lastSecondTime>= 1000 && dsHZ < maxhertz) {
          //     dsHZ  = dsHZ + 10;
          //     HZ = hertzToMilliseconds(dsHZ);
          //     lastSecondTime = currentMillis;
          //     // Serial.print("HZ: ");
          //     // Serial.println(HZ);
          //   }
           if (currentMillis - lastSendTime >= sendInterval) {
            // Serial.println(SharedSecret);
            message msg;
            msg.Angle = esp_random();
            PrevAngle = msg.Angle;
            msg.Velocity = esp_random();
            PrevVelocity = msg.Velocity;
            addChecksumToMessage(&msg);  // Calculate checksum on the plain message

            encryptmsg((uint8_t *)&msg, sizeof(message), SharedSecret);  // Encrypt the whole message including the checksum
            // Serial.print("Sent: ");
            // Serial.print(PrevAngle);
            // Serial.print(" ");
            // Serial.print(PrevVelocity);
            // Serial.println();
            // lastSendTime = currentMillis;
            esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(message));
          }
        }
        if (leaderState == FOLLOWER) {
          if (currentMillis - lastSecondTime >= 1000) {
            Serial.print("Packets per second: ");
            Serial.println( packetsPerSecondCounter);
            Serial.print("Packetloss per second: ");
            Serial.println( packetloss);
            
            
            lastSecondTime = currentMillis;
            lastSecondCounter = packetsPerSecondCounter;
            packetsPerSecondCounter = 0;
            packetloss = 0;
          }

        }




        break;
      }
  }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  if (status != ESP_NOW_SEND_SUCCESS) {
    if (deviceState == CONNECTED) {
      deviceState = IDLE;
    }
    Serial.println("Failed to send data");
  }
}


// Function to encrypt a message
void encryptmsg(uint8_t *data, size_t dataSize, int secretkey) {
  for (size_t i = 0; i < dataSize; i++) {
        data[i] = data[i] ^ (static_cast<uint8_t>(secretkey >> (8 * (i % sizeof(int)))));
    }
}

// Function to decrypt a message
void decryptmsg(uint8_t *data, size_t dataSize, int secretkey) {
  for (size_t i = 0; i < dataSize; i++) {
        data[i] = data[i] ^ (static_cast<uint8_t>(secretkey >> (8 * (i % sizeof(int)))));
    }
}

// Function to add a checksum to a message
void addChecksumToMessage(message *msg) {
  msg->checksum = crc32(msg, sizeof(message) - sizeof(msg->checksum));
}

uint32_t crc32(const void *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  const uint8_t *bytes = static_cast<const uint8_t *>(data);

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

  return (u_int64_t)result;
}

// publickey generate function
int PublicKey() {
  if (PrivateKey == 0) {
    PrivateKey = esp_random() % Prime;
  }
  // Serial.println("PublicKey");
  // Serial.println(PrivateKey);
  // Serial.println(Generator);
  // Serial.println(Prime);
  // Serial.println(modularExponentiation(Generator, PrivateKey, Prime));
  return modularExponentiation(Generator, PrivateKey, Prime);
}

// secretkey generate function
int SecretKey(int rPublicKey) {
  // Serial.println("SecretKey");
  if (PrivateKey == 0) {
    PrivateKey = esp_random() % Prime;
  }
  // Serial.println(PrivateKey);
  // Serial.println(rPublicKey);
  // Serial.println(Prime);
  // Serial.println(modularExponentiation(rPublicKey, PrivateKey, Prime));
  return modularExponentiation(rPublicKey, PrivateKey, Prime);
}

void handleKeyExchange(const uint8_t *incomingData, int len, uint8_t *peerAddress, bool &myKeySent, DeviceState deviceState, String key) {
  if (len == sizeof(Keymsg)) {
    Serial.println("Received public key.");
    Keymsg *kmsg = (Keymsg *)incomingData;
    if (kmsg->checksum == crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum))) {
      Serial.println(kmsg->PublicKey);
      int skey = kmsg->PublicKey;
      SharedSecret = SecretKey(skey);
      Serial.println("Verified key public key.");
      if (!myKeySent) {
        kmsg->PublicKey = PublicKey();
        // Serial.println("Sent public key.");
        // Serial.println(kmsg->PublicKey);
        kmsg->checksum = crc32(kmsg, sizeof(Keymsg) - sizeof(kmsg->checksum));
        esp_now_send(peerAddress, (uint8_t *)kmsg, sizeof(Keymsg));

        Serial.println("Key exchange complete.");
        myKeySent = true;
        keyEstablished = true;
        Serial.println(SharedSecret);
        deviceState = CONNECTED;
      } else {
        Serial.println("Key exchange complete.");
        keyEstablished = true;
        Serial.println(SharedSecret);
        deviceState = CONNECTED;
      }
    } else {
      Serial.println("Checksum verification failed for received public key.");
    }
  } else {
    Serial.println("Wrong size for key message.");
  }
}



void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Handle incoming data based on the current state
  // Serial.println("Data received");

  switch (deviceState) {
    case IDLE:
      // Handle "hello" message
      if (len == sizeof(HelloMsg)) {
        HelloMsg *hmsg = (HelloMsg *)incomingData;
        Serial.println("Received hello message");
        if (hmsg->checksum == crc32(hmsg, sizeof(HelloMsg) - sizeof(hmsg->checksum))) {

          // Send back an "acknowledgement" message
          AckMsg msg;
          msg.ranack = 31108;
          msg.checksum = crc32((uint8_t *)&msg, sizeof(AckMsg) - sizeof(msg.checksum));
          leaderState = FOLLOWER;
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
        AckMsg *amsg = (AckMsg *)incomingData;
        if (amsg->checksum == crc32(amsg, sizeof(AckMsg) - sizeof(amsg->checksum))) {
          Serial.println("ACK verified");
          deviceState = KEY_EXCHANGE;
          leaderState = LEADER;
        } else {
          Serial.println("Checksum verification failed for received ack message.");
        }
      } else {
        Serial.println("Wrong size for ack message.");
      }

      break;
    case KEY_EXCHANGE:
      handleKeyExchange(incomingData, len, peerAddress, myKeySent, deviceState, "Key_exchange");

      if (deviceState != CONNECTED and keyEstablished == true) {
        deviceState = CONNECTED;
      }
      break;
    case WAITING_KEY_EXCHANGE:
      handleKeyExchange(incomingData, len, peerAddress, myKeySent, deviceState, "waiting_Key_exchange");

      if (deviceState != CONNECTED and keyEstablished == true) {
        deviceState = CONNECTED;
      }
      break;

    case CONNECTED:
      if (len == sizeof(retmsg) and leaderState == LEADER) {
      
        retmsg *rmsg = (retmsg *)incomingData;

        Serial.println("Received ret message");
        message msg;
        msg.Angle = PrevAngle;
        msg.Velocity = PrevVelocity;
        encryptmsg((uint8_t *)&msg, sizeof(message), SharedSecret);
        addChecksumToMessage(&msg);
        Serial.print("Retransmitted Sent: ");
        Serial.print(msg.Angle);
        Serial.print(" ");
        Serial.print(msg.Velocity);
        Serial.println();
        esp_now_send(peerAddress, (uint8_t *)&msg, sizeof(message));
      }
      if (len == sizeof(message) && leaderState == FOLLOWER) {
        message *msg = (message *)incomingData;
        // Serial.println("Received message");


        // Verify checksum after decrypting
        decryptmsg((uint8_t *)msg, sizeof(message), SharedSecret);  // Decrypt the whole message including the checksum
       
        if (msg->checksum == crc32(msg, sizeof(message) - sizeof(msg->checksum))) {
          packetsPerSecondCounter++;
          // Serial.println(SharedSecret);
          // Serial.print("Received: ");
          // Serial.print(msg->Angle);
          // Serial.print(" ");
          // Serial.print(msg->Velocity);
          // Serial.println();
          // (Place for sending back an acknowledgment if needed)
        } else {
          // Serial.println("Checksum verification failed for received message.");
          packetloss++;
          retmsg rmsg;
          rmsg.ret = true;
          esp_now_send(peerAddress, (uint8_t *)&rmsg, sizeof(retmsg));
        }
      }


      break;
  }
}

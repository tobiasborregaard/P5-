#include <esp_now.h>
#include <WiFi.h>



enum DeviceState {
  IDLE,
  WAITING_FOR_ACK,
  KEY_EXCHANGE,
  CONNECTED,
  WAITING_KEY_EXCHANGE,

};


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

//======initializing variables==========
// =====Diffie hellmann key exchange=========

bool myKeySent = false;
bool keyEstablished = false;  // Flag to check if the key has been established
bool isInitiator = false;

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

void addChecksumToMessage(message *msg) {
  msg->checksum = crc32(msg, sizeof(message) - sizeof(msg->checksum));
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
  // Serial.println("SecretKey");
  if (PrivateKey == 0) {
    PrivateKey = esp_random() % Prime;
  }

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

void resetKeyExchange() {
  myKeySent = false;
  keyEstablished = false;
  isInitiator = false;
  PrivateKey = 0;
  SharedSecret = 0;
  
}
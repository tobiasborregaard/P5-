#include <esp_now.h>
#include <WiFi.h>

struct message {
  double Angle;
  double Velocity;
  uint32_t checksum;
} __attribute__((packed));

struct Keymsg {
  int PublicKey;
  uint32_t checksum;
};

struct AckMsg {
  uint32_t ranack;
  uint32_t roger;
  uint32_t checksum;
};


//======initializing variables==========
//=====Diffie hellmann key exchange=========

bool myKeySent = false;
bool keyEstablished = false;  // Flag to check if the key has been established

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

// Function to add a checksum to a message
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

// Function to 
void resetKeyExchange() {
  myKeySent = false;
  keyEstablished = false;
  isInitiator = false;
  PrivateKey = 0;
  SharedSecret = 0; 
}
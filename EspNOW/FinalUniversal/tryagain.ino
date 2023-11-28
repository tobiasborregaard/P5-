#include <esp_now.h>
#include <WiFi.h>
#include <cmath>


uint8_t peerAddress1[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 }; // Updated to match TTGO: 3C:61:05:0B:BB:90
uint8_t peerAddress2[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 }; // Matches with LILYGO: 08:3A:F2:69:CF:64
#define CHANNEL 0

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
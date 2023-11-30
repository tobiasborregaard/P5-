
#include <cmath>
// test sizeof and sizeof() on structs

struct Keymsg {
    uint8_t key[32];
    uint8_t checksum;
};

struct message {
    uint8_t Angle;
    uint8_t Speed;
    uint8_t checksum;
};




void setup(){
    Serial.begin(115200);
    // Serial.println("Hello World");
    // Serial.println(sizeof(Keymsg));
    // Serial.println(sizeof(message));


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

}

void loop(){
    Serial.println(modularExponentiation(2, 3, 5));
    int a=(pow(2,3));
    int b= a%5;
    Serial.println(b);
    delay(1000);
}
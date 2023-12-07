
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
int modularExponentiation(u_int base, u_int exponent, u_int modulus) {
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
int prime = 707898413;
int generator = 2 ;

int prkey(){
 int PrivateKey = esp_random() % prime;
 
 return PrivateKey;
}
void loop(){
    Serial.println("this is cool");
    Serial.println(modularExponentiation(generator, prkey(), prime));
    Serial.println("this is not cool");

    long long a=(pow(generator,prkey()));
    long b= a%prime;
    Serial.println(b);
    delay(1000);
}
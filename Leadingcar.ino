#include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>



//===========Angle relateret inputs og variabler=================

#define ServoPin 27         // GPIO 4 on the ESP  //A1 pin på Arduino
float input_voltage = 0.0;


//===========Velocity  relateret inputs og variabler=================

#define OpticPin 17

double lasttime = 0;
double delaytimer = 5;

double speedTimeCal = 1000;
double speedLastTime = 0;

double calSpeed = 0;
int counter = 0;

int analog_value;
double Angle;


// Initialize TFT display
TFT_eSPI tft = TFT_eSPI();  // Corrected: added parentheses
BluetoothSerial SerialBT;

#define Width 135
#define Height 240


//hvis skærmen skal bruges til noget
void tftsetup(){
tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor((Width / 2)-38, Height / 2);
  tft.print(WiFi.macAddress());


}
// kan sende data til en computer med bluetooth
void bluetoothsetup(){
    SerialBT.begin("Der fuhrer"); //Bluetooth for leading car
    // Serial.



}
// esp now initialisering
void InitESP() {
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();  // Ensure we're not connected to any WiFi network (optional)

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW initialization successful");
  } else {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = CHANNEL;  // Set the peer channel
  peerInfo.encrypt = false;    // Data will not be encrypted

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
}

void setup() {
    Serial.begin(115200);
    Serial.print("Engine starting..");
    pinMode(ServoPin, INPUT);
    pinMode(OpticPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
    tftsetup();
    bluetoothsetup();
    InitESP(); 

}


void loop() {

  //======= Sampling og konverting til spænding======
  analog_value = analogRead(ServoPin);
  //bool latest_value = digitalRead(35);
  input_voltage = (analog_value * 4) / 4095.0;

  if (input_voltage < 0.1)   //Filtrering af shit measurement
  {
    input_voltage = 0.0;
  }
  AngleCalculator(input_voltage);
  int oldTime = millis();
  delay(1000);  //Afgørende delay for at nå at beregne en hastighed! //Skal nok tweakes lidt
  CarVelocity();
  //Serial.println("Counter: " + counter);
  //Serial.println("Latest value: " + latest_value);
}

//===========Angle funktion, tager servospænding som input og retunere vinkel på baggrund af funktion=================
void AngleCalculator(float V) {

  Angle = 39.8613 * V - 57.6003; // Relation mellem vinkel og spænding

  Serial.print("ServoSpænding: ");
  Serial.print(input_voltage);
  Serial.println(" V");

  Serial.print("Angle: ");
  Serial.print(Angle);
  Serial.println(" °");

}

//===========Pulse counter funktion - Kaldes ud fra interrupt=================

void Countup() {
  if (millis() - lasttime >= delaytimer) {
    counter++;
    //Serial.println(counter);
    lasttime = millis();
  }
}


//===========Velocity Funktion=================

void CarVelocity() {
  if (millis() - speedLastTime >= speedTimeCal ) {
    //Beregning i m/s
    calSpeed = ((counter * 3.656) / 100) / (speedTimeCal / 1000); //SpeedTimeCal er divideret med 1000 for omregning til sekunder.
    speedLastTime = millis();
    Serial.print("Speed is currently: ");
    Serial.print(calSpeed);
    Serial.println(" m/s");

    calSpeed *= 3.6;    // Konvertering til km/t
    Serial.print("Speed in km/h: ");
    Serial.print(calSpeed);
    Serial.println(" km/h");

    counter = 0;
  }
}


// Callback when data is received
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int data_len) {
  if (data_len == sizeof(message)) {
    const message* msg = reinterpret_cast<const message*>(incomingData);
    Serial.print("Received data from: ");
    for (int i = 0; i < 6; ++i) {
      if (mac_addr[i] < 16) {
        Serial.print("0"); // Print a leading zero if necessary
      }
      Serial.print(mac_addr[i], HEX);
      if (i < 5) {
        Serial.print(":"); // Print a colon after each byte except the last
      }
    }
    Serial.println(); // Print a newline after the MAC address

    Serial.print("Message: ");
    Serial.println(msg->text);
  } else {
    Serial.println("Received data size does not match expected message size.");
  }
}


void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendData(const message* dataToSend) {
  esp_err_t result = esp_now_send(peerAddress, (uint8_t*)dataToSend, sizeof(message));

  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}
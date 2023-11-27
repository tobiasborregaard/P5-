#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>



//===========Angle relateret inputs og variabler=================

#define ServoPin 27  // GPIO 4 on the ESP  //A1 pin på Arduino
float input_voltage = 0.0;


//===========Velocity  relateret inputs og variabler=================

#define OpticPin 17

double lasttime = 0;
double delaytimer = 5;

double speedTimeCal = 1000;
double speedLastTime = 0;

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval at which to run (milliseconds)

double calSpeed = 0;
int counter = 0;

int analog_value = 0;
double Angle;


#define CHANNEL 0
//ttgo
uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x45, 0x3D, 0xE8 };
// lilygo
//uint8_t peerAddress[] = { 0x08, 0x3A, 0xF2, 0x69, 0xCF, 0x64 };

esp_now_peer_info_t peerInfo;
// skal ændres så vi kan sende vores egen pakke struktur
struct message {
  double Angle;
  float Speed;
} __attribute__((packed));  // Ensure no padding is added to the struct

// Define an ISR for the timer
void IRAM_ATTR onTimer(void* arg) {
    CarVelocity();
}

void timerinit() {
    // Timer configuration
    esp_timer_create_args_t timer_args = {}; // Zero-initialize the struct
    timer_args.callback = &onTimer;
    timer_args.name = "MyTimer";

    esp_timer_handle_t my_timer_handle;
    if (esp_timer_create(&timer_args, &my_timer_handle) != ESP_OK) {
        // Handle error (e.g., print an error message)
    }

    // Start the timer with a 1-second period (1000000 microseconds)
    if (esp_timer_start_periodic(my_timer_handle, 1000000) != ESP_OK) {
        // Handle error (e.g., print an error message)
    }
}



// esp now initialisering
void InitESP() {
  WiFi.mode(WIFI_STA);
  // Serial.println(WiFi.macAddress());
  WiFi.disconnect();  // Ensure we're not connected to any WiFi network (optional)

  if (esp_now_init() == ESP_OK) {
  } else {
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = CHANNEL;  // Set the peer channel
  peerInfo.encrypt = false;    // Data will not be encrypted

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    // Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}


void wireless(bool state) {
  if (state == true) {
    InitESP();
  } else {
    WiFi.mode(WIFI_OFF);
    esp_now_deinit();
  }
}
void setup() {
  Serial.begin(115200);
  // Serial.print("Engine starting..");
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
  timerinit();
  InitESP();
}

void loop() {

  //======= Sampling og konverting til spænding======
  AngleCalculator();
 

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // showemilio(true);
    //Serial.println(analog_value);
    // Create a message to send
    message msgToSend = { Angle, calSpeed };
    String package = String(Angle, 2) + " , " + String(calSpeed, 2);
    Serial.println(package);  // Debug print to Serial Monitor


    
    sendData(&msgToSend);
    
  }
}



//===========Angle funktion, tager servospænding som input og retunere vinkel på baggrund af funktion=================
void AngleCalculator() {
  wireless(false);
  analog_value = analogRead(ServoPin);
  input_voltage = (analog_value * 4) / 4095.0;


  if (input_voltage < 0.1) {
    input_voltage = 0.0;
  }

  Angle = 39.8613 * input_voltage - 57.6003;  // Relation mellem vinkel og spænding


  wireless(true);
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
  if (millis() - speedLastTime >= speedTimeCal) {
    //Beregning i m/s
    calSpeed = ((counter * 3.656) / 100) / (speedTimeCal / 1000);  //SpeedTimeCal er divideret med 1000 for omregning til sekunder.
    speedLastTime = millis();

    calSpeed *= 3.6;  // Konvertering til km/t

    counter = 0;

  }
}


// Callback when data is received
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int data_len) {
  if (data_len == sizeof(message)) {
    const message* msg = reinterpret_cast<const message*>(incomingData);

  } else {
    Serial.println("Received data size does not match expected message size.");
  }
}


void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 
}

void sendData(const message* dataToSend) {
  esp_err_t result = esp_now_send(peerAddress, (uint8_t*)dataToSend, sizeof(message));

  if (result == ESP_OK) {
    // Serial.println("Data sent successfully");
  } else {
    // Serial.println("Error sending data");
  }
}



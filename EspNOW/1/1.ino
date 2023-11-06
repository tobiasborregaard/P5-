#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
typedef struct struct_message {
  char a[32];
} struct_message;

// Create a struct_message to hold incoming data
struct_message incomingData;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("listening");
  if (len == sizeof(struct_message)) {
    struct_message incoming;
    memcpy(&incoming, incomingData, sizeof(incoming));  // Make sure to copy the data into your struct
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Message: ");
    Serial.println(incoming.a);  // Now you access the a member correctly
  } else {
    Serial.println("Data received, but length does not match struct_message size.");
  }
}

void setup() {
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    esp_now_init();
    Serial.println("ESP-NOW Initialized");
  }

  
  // Get MAC Address
  Serial.println(WiFi.macAddress());
  
  // Once ESPNow is successfully Init, register the receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing to do here, data will be received via callback
}

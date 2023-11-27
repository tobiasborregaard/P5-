#include <Arduino.h>
#include <esp_timer.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();  
#define triggerPin1 25
#define echoPin1 26
#define triggerPin2 27
#define echoPin2 33

// Function to measure the distance
double measureDistance(int triggerPin, int echoPin) {
    double duration, distance;

    // Clear the trigger pin
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    // Set the trigger pin high for 10 microseconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    int64_t startMeasureTime = esp_timer_get_time();
    int64_t timeout = 23306; // Timeout for maximum distance

    // Wait for echo start
    while(digitalRead(echoPin) == LOW) {
        if (esp_timer_get_time() - startMeasureTime > timeout) {
            return -1; // -1 indicates no echo within expected time
        }
    }

    int64_t startMicros = esp_timer_get_time();
    // Measure how long the echo pin was high - from pulse start to end
    while(digitalRead(echoPin) == HIGH) {
        if (esp_timer_get_time() - startMicros > timeout) {
            return -1; // -1 indicates no echo within expected time
        }
    }
    int64_t endMicros = esp_timer_get_time();

    duration = endMicros - startMicros;

    // Calculate the distance
    distance = (duration * 0.0343) / 2; // Speed of sound wave divided by 2 (go and back)
    return distance;
}


//hvis skærmen skal bruges til noget
void tftsetup() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 80);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Text color, background color
  tft.setSwapBytes(true);
}

void setup() {
    Serial.begin(115200);
    
    pinMode(triggerPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(triggerPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
}

void loop() {
    // double distance1 = measureDistance(triggerPin1, echoPin1);
    double distance2 = measureDistance(triggerPin2, echoPin2);

    // Serial.print("Distance 1: ");
    Serial.println(distance2);
    // Serial.print(" cm. Distance 2: ");
    // Serial.print(distance2);
    // Serial.println(" cm.");

    delay(60); // Wait for a second before the next measurement to avoid signal interference
}

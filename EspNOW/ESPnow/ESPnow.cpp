#include "ESPnow.h"
#include <esp_now.h>
#include <WiFi.h>

// Static instance pointer
ESPnow* ESPnow::instance = nullptr;

// Constructor
ESPnow::ESPnow() {
    instance = this;  // Set the instance pointer
}

// begin method
bool ESPnow::begin() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        return false;  // Initialization failed
    }
    return true;
}

// sendtoall method
void ESPnow::sendtoall(String message, String mac_Address[5]) {
   for (int i = 0; i < 5; i++) {
        if (mac_Address[i].isEmpty() || mac_Address[i] == WiFi.macAddress()) {
            continue;
        }
        uint8_t addr[6];
        sscanf(mac_Address[i].c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
        sendto(message, addr);
    }
}

bool ESPnow::sendto(String message, uint8_t *mac) {
    esp_err_t result = esp_now_send(mac, (uint8_t *)message.c_str(), message.length());
    return (result == ESP_OK);
}

void ESPnow::onReceive(void(*callback)(String)) {
    _callback = callback;
    esp_now_register_recv_cb(onReceiveData);
}
void ESPnow::onReceiveData(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (!instance) return;
    
    String receivedMessage = "";
    for (int i = 0; i < len; i++) {
        receivedMessage += (char)incomingData[i];
    }

    instance->_callback(receivedMessage);
}

// Method to pair with a peer
bool ESPnow::addPeer(uint8_t *mac) {
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false; 

    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    return (addStatus == ESP_OK);
}


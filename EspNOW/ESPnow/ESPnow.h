#ifndef ESPnow_h
#define ESPnow_h


#include <esp_now.h>
#include <WiFi.h>

class ESPnow {
  public:
    ESPnow();
    bool begin();
    
    void sendtoall(String message, String mac_Address[5]);
    bool sendto(String message, uint8_t *mac);
    void onReceive(void(*callback)(String));
    String mac_Address[5];
    bool addPeer(uint8_t *mac);
    
  private:
  static ESPnow* instance;
    void(*_callback)(String);
    static void onReceiveData(const uint8_t *mac, const uint8_t *incomingData, int len);


  
};

#endif

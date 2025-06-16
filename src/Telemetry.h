#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

typedef struct {
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
  } IMUPacket;

class Telemetry {

    public :
        void init();                            // Init & configure SiK
        void enterCommandMode();                // Enter AT command mode
        void sendATCommand(const char* cmd);    // Send AT command
        void setAirDataRate(uint8_t rate);      // Set air data rate
        void setBaudRate(uint32_t baud);        // Set baud rate

        void sendMessage(const char* msg);                      // Send data message
        void sendBinary(const uint8_t* data, size_t len);       // Binary payload
        bool receiveMessage(char* buffer, size_t maxLen);       // Receive text

        size_t receiveBinary(uint8_t* buffer, size_t maxLen);   // Binary receive
        void buildPacket(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* outBuffer); 
        bool receivePacket(uint8_t* type, uint8_t* payload, uint8_t* length);
    
    private :

};

#endif // TELEMETRY_H

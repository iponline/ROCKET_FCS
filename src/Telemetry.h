#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

void init();                            // Init & configure SiK
void enterCommandMode();                // Enter AT command mode
void sendATCommand(const char* cmd);    // Send AT command
void setAirDataRate(uint8_t rate);      // Set air data rate
void setBaudRate(uint32_t baud);        // Set baud rate

void sendMessage(const char* msg);                      // Send data message
void sendBinary(const uint8_t* data, size_t len);       // Binary payload
bool receiveMessage(char* buffer, size_t maxLen);       // Receive text

size_t receiveBinary(uint8_t* buffer, size_t maxLen);   // Binary receive
void creatPacket(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* outBuffer); 
bool receivePacket(uint8_t* type, uint8_t* payload, uint8_t* length);

#endif // TELEMETRY_H

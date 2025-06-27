#pragma once
#include <Arduino.h>

// Define your EEPROM address layout here
#define EEPROM_ADDR_FLOAT      0
#define EEPROM_ADDR_UINT16     (EEPROM_ADDR_FLOAT + sizeof(float))
#define EEPROM_ADDR_PIDGAINS   (EEPROM_ADDR_UINT16 + sizeof(uint16_t))

// Example struct
// struct PIDGains {
//     float kp, ki, kd;
// };

// Template declarations
template <typename T>
void EEPROM_write(int address, const T& value);

template <typename T>
void EEPROM_read(int address, T& value);

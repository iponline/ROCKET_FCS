#include "EEPROMHelper.h"
#include <EEPROM.h>

template <typename T>
void EEPROM_write(int address, const T& value) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); ++i) {
        EEPROM.update(address + i, p[i]);
    }
}

template <typename T>
void EEPROM_read(int address, T& value) {
    uint8_t* p = reinterpret_cast<uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); ++i) {
        p[i] = EEPROM.read(address + i);
    }
}


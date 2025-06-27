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

// Explicit template instantiation for the types you use
//template void EEPROM_write<float>(int, const float&);
//template void EEPROM_read<float>(int, float&);

//template void EEPROM_write<uint16_t>(int, const uint16_t&);
//template void EEPROM_read<uint16_t>(int, uint16_t&);

//template void EEPROM_write<PIDGains>(int, const PIDGains&);
//template void EEPROM_read<PIDGains>(int, PIDGains&);

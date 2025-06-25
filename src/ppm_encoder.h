#pragma once
#include <Arduino.h>

class PPMEncoder {
public:
    PPMEncoder(uint8_t pin, uint8_t numChannels, uint16_t frameLength = 22500, uint16_t pulseLength = 300);
    void begin();
    void setChannel(uint8_t ch, uint16_t value); // Not thread safe, caller must ensure safety!
    void setAllChannels(const uint16_t* values, uint8_t count);

private:
    static void outputTask(void* arg);
    void outputFrame();

    uint8_t _pin;
    uint8_t _numChannels;
    uint16_t _frameLength;
    uint16_t _pulseLength;
    uint16_t* _channels; // No longer volatile
};

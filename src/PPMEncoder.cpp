#include "PPMEncoder.h"
#include "arduino_freertos.h"

PPMEncoder::PPMEncoder(uint8_t pin, uint8_t numChannels, uint16_t frameLength, uint16_t pulseLength)
    : _pin(pin), _numChannels(numChannels), _frameLength(frameLength), _pulseLength(pulseLength)
{
    _channels = new uint16_t[_numChannels];
    for(uint8_t i = 0; i < _numChannels; ++i) _channels[i] = 1500;
}

void PPMEncoder::begin() {

    pinMode(_pin, arduino::OUTPUT);
    digitalWriteFast(_pin, arduino::HIGH);
    xTaskCreate(PPMEncoder::outputTask, "PPMOut", 1024, this, 2, NULL);

}

void PPMEncoder::setChannel(uint8_t ch, uint16_t value) {

    if(ch < _numChannels)
        _channels[ch] = value;

}


void PPMEncoder::setAllChannels(const uint16_t* values, uint8_t count) {

    for(uint8_t i = 0; i < _numChannels && i < count; ++i)
        _channels[i] = values[i];
}

void PPMEncoder::outputTask(void* arg) {
    PPMEncoder* self = (PPMEncoder*)arg;
    while(1) {
        self->outputFrame();
    }
}


void PPMEncoder::outputFrame() {
    
    uint32_t time_used = 0;

    for(uint8_t i = 0; i < _numChannels; ++i) {
        digitalWriteFast(_pin, arduino::LOW);
        delayMicroseconds(_pulseLength);
        time_used += _pulseLength;

        digitalWriteFast(_pin, arduino::HIGH);
        uint16_t high_time = _channels[i] - _pulseLength;
        delayMicroseconds(high_time);
        time_used += high_time;
    }

    // SYNC PULSE
    uint16_t sync_gap = _frameLength - time_used;
    digitalWriteFast(_pin, arduino::LOW);
    delayMicroseconds(_pulseLength);
    digitalWriteFast(_pin, arduino::HIGH);
    delayMicroseconds(sync_gap - _pulseLength);
}

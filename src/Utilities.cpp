#include "Utilities.h"

void handleLED(uint8_t pin, float frequencyHz = 0.0f, bool staticOn = true) {
    static bool initialized = false;
    static bool ledState = false;
    static unsigned long lastToggle = 0;

    // One-time setup
    if (!initialized) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        initialized = true;
    }

    if (frequencyHz <= 0.0f) {
        // Static mode
        digitalWrite(pin, staticOn ? HIGH : LOW);
    } else {
        // Blinking mode
        unsigned long now = millis();
        float period = 1000.0f / frequencyHz / 2.0f;
        if (now - lastToggle >= period) {
            ledState = !ledState;
            digitalWrite(pin, ledState);
            lastToggle = now;
        }
    }
}
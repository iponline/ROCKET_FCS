#include "PPMcontrol.h"
#include <PPMReader.h>

// Keep PPM in one TU to avoid duplicate symbols
static PPMReader ppm(PPM_INPUT_PIN, NUM_CHANNELS);

// Defaults
static int chDefaults[3] = {1500, 1500, 1000};

void ppmInit() {
    // Nothing else needed: PPMReader starts on construction.
    // Optionally you could re-attach interrupts etc. if required.
}

void ppmRead3(int& ch1_us, int& ch2_us, int& ch3_us) {
    // Channels are typically 1-indexed in many RC setups:
    // ch1: roll, ch2: pitch, ch3: throttle
    int r = ppm.latestValidChannelValue(1, chDefaults[0]);
    int p = ppm.latestValidChannelValue(2, chDefaults[1]);
    int t = ppm.latestValidChannelValue(3, chDefaults[2]);

    // basic sanity clamp
    auto clamp = [](int v){ return v < 800 ? 800 : (v > 2200 ? 2200 : v); };

    ch1_us = clamp(r);
    ch2_us = clamp(p);
    ch3_us = clamp(t);

    // refresh defaults so subsequent “no new frame” states stay stable
    chDefaults[0] = ch1_us;
    chDefaults[1] = ch2_us;
    chDefaults[2] = ch3_us;
}

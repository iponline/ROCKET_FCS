/**
 * @file    main.cpp
 * @brief   Rocket Flight control for Teensy boards
 * @author  Sasit Chuasomboon
 * @date    11.06.2025
 */

 #include "arduino_freertos.h"
#include "semphr.h"
#include "avr/pgmspace.h"
#include "IMU.h"
#include "types.h"
#include "core_pins.h"

IMU imu;
FilteredData fdata;
IMUCalibration cal;

// ไม่ต้องใช้ Semaphore/ISR แล้ว

static void IMU_read(void*) {
    
    imu.begin();
    imu.calibrate(cal, 5000, 2);

    static uint32_t lastTick = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 ms = 50 Hz

    // เก็บเวลาเริ่มต้น
    lastTick = millis();

    while (true) {
        uint32_t now = millis();
        double dt = (now - lastTick) / 1000.0;
        if (dt <= 0.0) dt = 0.001;
        lastTick = now;

        imu.KalmanData(fdata, dt);

        Serial.print(millis());
        Serial.print(",");
        Serial.print(fdata.kalAngleX, 3);
        Serial.print(",");
        Serial.println(fdata.kalAngleY, 3);

        // Delay ให้ครบ 50 Hz
        vTaskDelay(xFrequency);
    }
}

FLASHMEM __attribute__((noinline)) void setup() {
    delay(2000);
    Serial.begin(115200);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(IMU_read, "task1", 256, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}

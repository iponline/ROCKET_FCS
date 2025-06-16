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
#include "Telemetry.h"
#include "PID.h"
#include "Servo.h"

IMU imu;
Telemetry telem;
FilteredData fdata;
IMUCalibration cal;

QueueHandle_t imuQueue;

Attitude attitude;          // เก็บค่า Pitch, Roll ล่าสุด (ใช้แชร์ข้าม Task)
SemaphoreHandle_t attitudeMutex;  // Mutex สำหรับป้องกันข้อมูลชนกัน

Servo servoPitch, servoRoll;

PID pidPitch(2.0, 0.5, 0.1);
PID pidRoll(2.0, 0.5, 0.1);

static void IMU_read(void*) {

    RawIMUData rawData;
    IMU_Data imuData;

    imu.begin();  // เริ่มการทำงานของ MPU6050
    imu.calibrate(cal, 5000, 2); // ทำการ Calibrate IMU

    static uint32_t lastTick = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50 ms = 20 Hz

    // เก็บเวลาเริ่มต้น
    lastTick = millis();

    while (true) {

        uint32_t now = millis();
        double dt = (now - lastTick) / 1000.0;
        if (dt <= 0.0) dt = 0.001;              // ป้องกันไม่ให้ dt เป็น 0 
        lastTick = now;

        imu.KalmanData(fdata, dt);              // คำนวณหา Pitch และ Roll ผ่าน Kalman filter

        Serial.print(millis());
        Serial.print(",");
        Serial.print(fdata.kalAngleX, 3);
        Serial.print(",");
        Serial.println(fdata.kalAngleX, 3);

        imu.readRaw(rawData);
        imu.IMUData(imuData, rawData);

        xQueueSend(imuQueue, &imuData, 0); // Non-blocking send
        //vTaskDelay(interval);

        // เขียนค่า attitude แบบ thread-safe
        xSemaphoreTake(attitudeMutex, portMAX_DELAY);
        attitude.pitch = fdata.kalAngleX;
        attitude.roll  = fdata.kalAngleX;
        xSemaphoreGive(attitudeMutex);

        // Delay ให้ครบ 20 Hz
        vTaskDelay(xFrequency);
    }
}

void telemetryTaskTX(void* pvParameters) {

    IMU_Data imuData;
    uint8_t payload[12];
    uint8_t packet[64];
  
    while (1) {
      if (xQueueReceive(imuQueue, &imuData, pdMS_TO_TICKS(100))) {

        memcpy(payload + 0, &imuData.ax, 2);
        memcpy(payload + 2, &imuData.ay, 2);
        memcpy(payload + 4, &imuData.az, 2);
        memcpy(payload + 6, &imuData.gx, 2);
        memcpy(payload + 8, &imuData.gy, 2);
        memcpy(payload + 10, &imuData.gz, 2);
  
        telem.buildPacket(0x10, (const char*)payload, 12, (char*)packet);
        telem.sendBinary((uint8_t*)packet, 3 + 12 + 1);
      }
    }
  }

void telemetryTaskRX(void* pvParameters) {

    uint8_t type;
    uint8_t payload[256];
    uint8_t len;
  
    while (1) {

      if (telem.receivePacket(&type, payload, &len)) {
        Serial.print("RX Packet Type: ");
        Serial.println(type);
        Serial.print("Payload: ");

        for (uint8_t i = 0; i < len; i++) {

          Serial.write(payload[i]);

        }
        Serial.println();
      }
  
      vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
    }
}

static void PID_Control(void*) {
    
    float setpointPitch = 0, setpointRoll = 0;
    pidPitch.setLimits(-45, 45);   // สมมติ servo คุมได้ -45 ถึง +45 องศา
    pidRoll.setLimits(-45, 45);

    while (1) {
        Attitude curAtt;
        // อ่าน attitude แบบ thread-safe
        xSemaphoreTake(attitudeMutex, portMAX_DELAY);
        curAtt = attitude;
        xSemaphoreGive(attitudeMutex);

        float dt = 0.02; // 20 ms = 50Hz

        float outPitch = pidPitch.compute(setpointPitch, curAtt.pitch, dt);
        float outRoll  = pidRoll.compute(setpointRoll, curAtt.roll, dt);

        // สมมุติ Servo กลางคือ 90°
        servoPitch.write(90 + outPitch);
        servoRoll.write(90 + outRoll);

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}
  

FLASHMEM __attribute__((noinline)) void setup() {

    delay(2000);
    Serial.begin(115200);
    telem.init();

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    imuQueue = xQueueCreate(10, sizeof(IMUPacket)); // Allocates a memory buffer for up to 10 IMUPacket structs.

    xTaskCreate(IMU_read, "IMU", 1024, nullptr, 3, nullptr);
    xTaskCreate(telemetryTaskTX, "TX", 1024, NULL, 2, NULL);
    xTaskCreate(telemetryTaskRX, "RX", 1024, NULL, 1, NULL);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}

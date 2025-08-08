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
#include "Arduino.h"
#include "PPMcontrol.h"   
#include "def.h"

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif



IMU imu;
Telemetry telem;
FilteredData fdata;
IMUCalibration cal;

QueueHandle_t imuQueue;
Attitude attitude;          // เก็บค่า Pitch, Roll ล่าสุด (ใช้แชร์ข้าม Task)
SemaphoreHandle_t attitudeMutex,setpointMutex;  // Mutex สำหรับป้องกันข้อมูลชนกัน
volatile float setpointPitch = 0, setpointRoll = 0;

Servo servoPitch, servoRoll;
//ppmServoRoll, ppmServoPitch, ppmServoThrottle;

const float STABILITY_BAND = 5.0;  // degrees
PID pidPitch(0.8, 0.1, 0.1);
PID pidRoll(0.8, 0.1, 0.1);


static void IMU_read(void*) {

    //RawIMUData rawData;
    //IMU_Data imuData;
    FilteredData fdata;

    imu.begin();  
    imu.calibrate(cal, 5000, 2); // ทำการ Calibrate IMU
    imu.EKFInitQ();              // **เริ่มต้น EKF Quaternion**

    static uint32_t lastTick = 0;
    //const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 ms = 1000 Hz

    lastTick = millis();

    while (true) {

        uint32_t now = millis();
        double dt = (now - lastTick) / 1000.0;
        if (dt <= 0.0) dt = 0.001;              // ป้องกันไม่ให้ dt เป็น 0 
        lastTick = now;

        // ---- คำนวณ Attitude (Quaternion EKF) ----
        if (imu.EKFQuaternionData(fdata, dt)) { 
            // ส่งออกค่า roll/pitch/yaw ทุก 10ms (100Hz) ที่ผ่าน LPF
            //Serial.print(millis());
            //Serial.print(",");
            //Serial.print(fdata.roll, 2);
            //Serial.print(",");
            //Serial.println(fdata.pitch, 2);
            //Serial.print(",");
            //Serial.println(fdata.yaw, 2);

            // เขียนค่า attitude แบบ thread-safe
            xSemaphoreTake(attitudeMutex, portMAX_DELAY);
            //Serial.println("In IMU xSemaphore!!!!");
            attitude.pitch = fdata.pitch;  // degree -> convert double to float
            attitude.roll  = fdata.roll;   // degree
            attitude.yaw   = fdata.yaw;    // degree
            xSemaphoreGive(attitudeMutex);
        }

        // ---- IMU Raw/Calibrated Data ส่งผ่าน queue ----
        //imu.readRaw(rawData);
        //imu.IMUData(imuData, rawData);
        //QueueSend(imuQueue, &imuData, 0); // Non-blocking send

        //vTaskDelay(xFrequency); // Delay ให้ครบ 1ms (1000Hz)
    }
}

void telemetryTaskTX(void* pvParameters) {

    float att_temp[3];      // roll, pitch, yaw
    uint8_t payload[12];
    uint8_t packet[64];

    while (1) {
        // อ่าน attitude (roll, pitch, yaw) แบบ thread-safe
        xSemaphoreTake(attitudeMutex, portMAX_DELAY);
        att_temp[0] = attitude.roll;
        att_temp[1] = attitude.pitch;
        att_temp[2] = attitude.yaw;
        xSemaphoreGive(attitudeMutex);

        // Pack ค่า roll, pitch, yaw (float, 4 bytes) ลง payload
        memcpy(payload + 0, &att_temp[0], 4);   // roll
        memcpy(payload + 4, &att_temp[1], 4);   // pitch
        memcpy(payload + 8, &att_temp[2], 4);   // yaw

        // สร้าง packet และส่งออก
        telem.buildPacket(0x10, (const uint8_t*)payload, 12, (uint8_t*)packet);
        telem.sendBinary((uint8_t*)packet, 3 + 12 + 1); // header + payload + checksum

        vTaskDelay(pdMS_TO_TICKS(10)); // ส่งออกทุก 10ms (100Hz) ปรับได้ตามต้องการ
    }
}


  void telemetryTaskRX(void* pvParameters) {

    uint8_t type;
    uint8_t payload[256];
    uint8_t len;

    while (1) {
        if (telem.receivePacket(&type, payload, &len)) {
            switch (type) {

                case 0x20: // Set PID gains (6 floats = 24 bytes)
                if (len >= 24) {

                    float pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd;
                    memcpy(&pitch_kp, payload,      4);
                    memcpy(&pitch_ki, payload + 4,  4);
                    memcpy(&pitch_kd, payload + 8,  4);
                    memcpy(&roll_kp,  payload + 12, 4);
                    memcpy(&roll_ki,  payload + 16, 4);
                    memcpy(&roll_kd,  payload + 20, 4);

                    pidPitch.setTunings(pitch_kp, pitch_ki, pitch_kd); // Optionally use mutex if shared
                    pidRoll.setTunings(roll_kp, roll_ki, roll_kd);
                }
                break;

                case 0x21: // Set setpoint
                    if (len >= 8) {

                        float newPitch, newRoll;
                        memcpy(&newPitch, payload, 4);
                        memcpy(&newRoll, payload + 4, 4);

                        xSemaphoreTake(setpointMutex, portMAX_DELAY);
                        setpointPitch = newPitch;
                        setpointRoll = newRoll;
                        xSemaphoreGive(setpointMutex);
                    }
                    break;
                
                case 0x40:   
                    if (len >= 1) {
                        if (payload[0] == 0x02) {
                            //ppmControlEnabled = true;
                        } else if (payload[0] == 0x00) {
                            //ppmControlEnabled = false;
                        }
                    }
                    break;

                case 0x50: // command FCC 
                    if (len >=1){

                        

                    }
                // ... other command cases
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield
    }
}

// static void PID_Control(void*) {
//     Serial.println("PID Task started!");
//     int counter = 0;
//     while (1) {
//         Serial.print("[PID] Tick: ");
//         Serial.println(counter++);
//         vTaskDelay(pdMS_TO_TICKS(1000));  // slower to reduce USB congestion
//     }
// }



static void PID_Control(void*) {
    
    //float setpointPitch = 0, setpointRoll = 0;

    servoRoll.attach(6);
    servoPitch.attach(7);

    pidPitch.setLimits(-90, 90);   // สมมติ servo คุมได้ -30 ถึง +30 องศา
    pidRoll.setLimits(-90, 90);

    Serial.println("PID_Control: starting PID control loop...");

    while (1) {

        Attitude curAtt;
        //อ่าน attitude แบบ thread-safe
        xSemaphoreTake(attitudeMutex, portMAX_DELAY);
        curAtt = attitude;
        xSemaphoreGive(attitudeMutex);

        

        // Serial.print(attitude.pitch);
        // Serial.print(",");
        // Serial.print(attitude.roll);
        // Serial.println();

        float dt = 0.001; // 1 ms = 1000Hz

        // Compute errors
        float errorPitch = setpointPitch - curAtt.pitch;
        float errorRoll  = setpointRoll  - curAtt.roll;

        // Optionally, zero output if within stability band
        float outPitch = 0, outRoll = 0;
        if (fabs(errorPitch) > STABILITY_BAND)
            outPitch = pidPitch.compute(setpointPitch, curAtt.pitch, dt);

        if (fabs(errorRoll) > STABILITY_BAND)
            outRoll = pidRoll.compute(setpointRoll, curAtt.roll, dt);

        // Serial.print(outPitch);
        // Serial.print(",");
        // Serial.print(outRoll);
        // Serial.println();

        // สมมุติ Servo กลางคือ 90°

        // Send PID output to servo (centered at 90)
        float pitchCommand = constrain(90 - outPitch, 0, 180);
        float rollCommand  = constrain(90 - outRoll,  0, 180);
        servoPitch.write(pitchCommand);
        servoRoll.write(rollCommand);
        //servoPitch.write(90 + outPitch);
        //servoRoll.write(90 + outRoll);

        vTaskDelay(pdMS_TO_TICKS(10)); // 1000Hz

        // // Return servos to neutral     
        // servoPitch.write(90);
        // servoRoll.write(90);

        // // Wait for PID effect to apply, then reset to center
        // vTaskDelay(pdMS_TO_TICKS(10));  // Apply correction briefly (10ms)
    }
}

void taskMonitor(void*) {

    while (1) {
        Serial.println("[Monitor] I'm alive");
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    // Create mutex before tasks use it
    attitudeMutex = xSemaphoreCreateMutex();
    if (attitudeMutex == NULL) {
        Serial.println("Failed to create attitudeMutex!");
        while (1); // halt
    }

    // Create mutex before tasks use it
    setpointMutex = xSemaphoreCreateMutex();
    if (setpointMutex == NULL) {
        Serial.println("Failed to create setpointMutex!");
        while (1); // halt
    }

    xTaskCreate(IMU_read, "IMU", 1024, nullptr, 3, nullptr);
    xTaskCreate(PID_Control, "PID", 2048, nullptr, 3, nullptr);
    // if (xTaskCreate(PID_Control, "PID", 2048, nullptr, 2, nullptr) != pdPASS) {
    //     Serial.println("Failed to start PID task!");
    // }
    //xTaskCreate(taskMonitor, "Monitor", 1024, nullptr, 1, nullptr);
    xTaskCreate(telemetryTaskTX, "TX", 1024, NULL, 3, NULL);
    //xTaskCreate(ppmControlTask, "PPM", 512, NULL, 1, NULL);
    //xTaskCreate(telemetryTaskRX, "RX", 1024, NULL, 1, NULL);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}

/**
 * @file    main.cpp
 * @brief   Rocket Flight control for Teensy boards
 * @version beta stable 1.1 (with throttle status query support)
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
 #include <EEPROM.h>
 #include "Utilities.h"
 
 #ifndef constrain
 #define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
 #endif
 
 // ---------- Defaults (used if EEPROM looks invalid) ----------
 static constexpr float DEF_PKP = 0.8f;
 static constexpr float DEF_PKI = 0.1f;
 static constexpr float DEF_PKD = 0.1f;
 static constexpr float DEF_RKP = 0.8f;
 static constexpr float DEF_RKI = 0.1f;
 static constexpr float DEF_RKD = 0.1f;
 static constexpr float DEF_SETPOINT_PITCH = 0.0f;
 static constexpr float DEF_SETPOINT_ROLL  = 0.0f;
 
 // ---------- EEPROM layout (8 floats = 32 bytes) ----------
 enum {
   EE_PKP = 0,     // Pitch Kp
   EE_PKI = 1,     // Pitch Ki
   EE_PKD = 2,     // Pitch Kd
   EE_RKP = 3,     // Roll Kp
   EE_RKI = 4,     // Roll Ki
   EE_RKD = 5,     // Roll Kd
   EE_SP_PITCH = 6,// Setpoint Pitch
   EE_SP_ROLL  = 7 // Setpoint Roll
 };
 static inline int eeAddr(int idx) { return idx * 4; } // each float is 4 bytes
 
 // ----------------- Globals -----------------
 IMU imu;
 Telemetry telem;
 FilteredData fdata;
 IMUCalibration cal;
 
 QueueHandle_t imuQueue;
 Attitude attitude;                            // last Pitch, Roll, Yaw
 SemaphoreHandle_t attitudeMutex, setpointMutex;
 volatile float setpointPitch = 0.0f, setpointRoll = 0.0f;
 
 Servo servoPitch, servoRoll, servoThrottle;
 
 const float STABILITY_BAND = 5.0f;
 PID pidPitch, pidRoll;
 
 // ---------- Stick Assist (manual) ----------
 volatile bool stickModeEnabled = false;           // toggled by 0x40
 static constexpr float STICK_MAX_ANGLE_DEG = 30.0f; // hard limit for stick -> setpoint
 static constexpr float STICK_SLEW_DPS      = 80.0f; // rate limit for setpoint changes
 static constexpr float STICK_DEADBAND_N    = 0.05f; // deadband for normalized input [-1..1]
 
 // ---------- Throttle status tracking ----------
 volatile uint8_t gThrottleStatus = 0x00;          // 0x00=KILL, 0x01=FULL

 // ---- New: task handles & mode state ----
TaskHandle_t hIMU = nullptr, hPID = nullptr, hTX = nullptr, hRX = nullptr, hSTICK = nullptr;
volatile bool controlPaused = false;          // true while IMU stream mode active

// ---- New: raw IMU snapshot (updated in IMU_read) ----
volatile float gAx = 0, gAy = 0, gAz = 0, gGx = 0, gGy = 0, gGz = 0;

// Reuse attitudeMutex to also guard raw IMU snapshot updates/reads

// ===== IMU Stream control (0x11 with 1-byte payload) =====
static constexpr uint8_t PKT_IMU_CTRL   = 0x11;  // UI->FCC control
static constexpr uint8_t IMU_CTRL_STOP  = 0x00;
static constexpr uint8_t IMU_CTRL_START = 0x01;
static constexpr uint8_t PKT_IMU_SAMPLE = 0x11;  // FCC->UI data packets (ax..gz)

// Stream parameters/state
static constexpr uint32_t IMU_STREAM_HZ = 100;   // stream rate in Hz
static volatile bool gImuStreamEnabled  = false;

// Forward decl
static void startImuStreamTask();
static TaskHandle_t imuStreamTaskHandle = nullptr;

 
 // ---------- Helpers ----------
 static bool saneGain(float g) {
   return isfinite(g) && fabsf(g) < 1e3f;
 }
 static bool saneSetpoint(float s) {
   return isfinite(s) && s >= -90.0f && s <= 90.0f;
 }
  
 // Kill throttle and record state
 static void forceThrottleKill() {
   if (!servoThrottle.attached()) servoThrottle.attach(10);   // throttle pin
   servoThrottle.write(0);            // 0° = KILL (ESC: use writeMicroseconds if needed)
   // servoThrottle.writeMicroseconds(600);
   gThrottleStatus = 0x00;
 }
 
 // Reply helper for throttle status query (opcode 0x40, subop 0x03)
 static void sendThrottleStatusReply() {
   // payload: [sub-op=0x03, status]
   uint8_t txPayload[2] = { 0x03, gThrottleStatus };
   uint8_t txPacket[3 + 2 + 1];
   telem.buildPacket(0x40, txPayload, 2, txPacket);
   telem.sendBinary(txPacket, sizeof(txPacket));
 }
 
 // ---------- EEPROM I/O ----------
 static void saveParamsToEEPROM() {
   float pkp, pki, pkd, rkp, rki, rkd;
   pidPitch.getTunings(pkp, pki, pkd);
   pidRoll.getTunings(rkp, rki, rkd);
 
   float spPitch, spRoll;
   xSemaphoreTake(setpointMutex, portMAX_DELAY);
   spPitch = setpointPitch;
   spRoll  = setpointRoll;
   xSemaphoreGive(setpointMutex);
 
   EEPROM.put(eeAddr(EE_PKP), pkp);
   EEPROM.put(eeAddr(EE_PKI), pki);
   EEPROM.put(eeAddr(EE_PKD), pkd);
   EEPROM.put(eeAddr(EE_RKP), rkp);
   EEPROM.put(eeAddr(EE_RKI), rki);
   EEPROM.put(eeAddr(EE_RKD), rkd);
   EEPROM.put(eeAddr(EE_SP_PITCH), spPitch);
   EEPROM.put(eeAddr(EE_SP_ROLL),  spRoll);
   Serial.println("[EEPROM] Saved PID gains and setpoints.");
 }
 
 static void applyDefaultsAndPersist() {
   pidPitch.setTunings(DEF_PKP, DEF_PKI, DEF_PKD);
   pidRoll.setTunings(DEF_RKP, DEF_RKI, DEF_RKD);
 
   xSemaphoreTake(setpointMutex, portMAX_DELAY);
   setpointPitch = DEF_SETPOINT_PITCH;
   setpointRoll  = DEF_SETPOINT_ROLL;
   xSemaphoreGive(setpointMutex);
 
   EEPROM.put(eeAddr(EE_PKP), DEF_PKP);
   EEPROM.put(eeAddr(EE_PKI), DEF_PKI);
   EEPROM.put(eeAddr(EE_PKD), DEF_PKD);
   EEPROM.put(eeAddr(EE_RKP), DEF_RKP);
   EEPROM.put(eeAddr(EE_RKI), DEF_RKI);
   EEPROM.put(eeAddr(EE_RKD), DEF_RKD);
   EEPROM.put(eeAddr(EE_SP_PITCH), DEF_SETPOINT_PITCH);
   EEPROM.put(eeAddr(EE_SP_ROLL),  DEF_SETPOINT_ROLL);
   Serial.println("[EEPROM] Reset to defaults and stored.");
 }
 
 static void loadParamsFromEEPROM() {
   float pkp, pki, pkd, rkp, rki, rkd, spPitch, spRoll;
 
   EEPROM.get(eeAddr(EE_PKP), pkp);
   EEPROM.get(eeAddr(EE_PKI), pki);
   EEPROM.get(eeAddr(EE_PKD), pkd);
   EEPROM.get(eeAddr(EE_RKP), rkp);
   EEPROM.get(eeAddr(EE_RKI), rki);
   EEPROM.get(eeAddr(EE_RKD), rkd);
   EEPROM.get(eeAddr(EE_SP_PITCH), spPitch);
   EEPROM.get(eeAddr(EE_SP_ROLL),  spRoll);
 
   bool gainsOK = saneGain(pkp) && saneGain(pki) && saneGain(pkd) &&
                  saneGain(rkp) && saneGain(rki) && saneGain(rkd);
   bool spOK = saneSetpoint(spPitch) && saneSetpoint(spRoll);
 
   if (gainsOK) {
     pidPitch.setTunings(pkp, pki, pkd);
     pidRoll.setTunings(rkp, rki, rkd);
   } else {
     Serial.println("[EEPROM] Gains invalid. Using defaults.");
     pidPitch.setTunings(DEF_PKP, DEF_PKI, DEF_PKD);
     pidRoll.setTunings(DEF_RKP, DEF_RKI, DEF_RKD);
   }
 
   xSemaphoreTake(setpointMutex, portMAX_DELAY);
   if (spOK) {
     setpointPitch = spPitch;
     setpointRoll  = spRoll;
   } else {
     Serial.println("[EEPROM] Setpoints invalid. Using defaults.");
     setpointPitch = DEF_SETPOINT_PITCH;
     setpointRoll  = DEF_SETPOINT_ROLL;
   }
   xSemaphoreGive(setpointMutex);
 
   Serial.println("[EEPROM] Loaded params.");
 }
 
 // ---------- IMU Task ----------
 static void IMU_read(void*) {
   FilteredData fdata;
 
   imu.begin();
   imu.calibrate(cal, 5000, 2);
   pinMode(20, 1);
   digitalWrite(20, 1);
   imu.EKFInitQ();
 
   static uint32_t lastTick = 0;
   lastTick = millis();
 
   while (true) {
     uint32_t now = millis();
     double dt = (now - lastTick) / 1000.0;
     if (dt <= 0.0) dt = 0.001;
     lastTick = now;
 
     if (imu.EKFQuaternionData(fdata, dt)) {
       xSemaphoreTake(attitudeMutex, portMAX_DELAY);
       attitude.pitch = fdata.pitch;
       attitude.roll  = fdata.roll;
       attitude.yaw   = fdata.yaw;
       xSemaphoreGive(attitudeMutex);
     }

     // 3) Also capture one fresh raw->scaled snapshot for IMU-stream (ax..gz)
     //    Note: IMUData() gives ax,ay,az in g, gx,gy,gz in deg/s with your current scaling
     RawIMUData raw;
     IMU_Data   scaled;
     if (imu.readRaw(raw)) {
      imu.IMUData(scaled, raw);
      xSemaphoreTake(attitudeMutex, portMAX_DELAY);
      gAx = scaled.ax; gAy = scaled.ay; gAz = scaled.az;  // [g]
      gGx = scaled.gx; gGy = scaled.gy; gGz = scaled.gz;  // [deg/s]
      xSemaphoreGive(attitudeMutex);
     }

     // Tight loop; sensor rate determines throughput
   }
 }

 static void ImuStreamTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_STREAM_HZ);
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (!gImuStreamEnabled) {
      vTaskDelay(pdMS_TO_TICKS(50));
      lastWake = xTaskGetTickCount();
      continue;
    }

    // Snapshot scaled IMU (ax..gz)
    float ax, ay, az, gx, gy, gz;
    if (xSemaphoreTake(attitudeMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      ax = gAx; ay = gAy; az = gAz;
      gx = gGx; gy = gGy; gz = gGz;
      xSemaphoreGive(attitudeMutex);
    } else {
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    // Send packet: type 0x11, payload 6 floats
    uint8_t txPayload[24];
    memcpy(txPayload + 0,  &ax, 4);
    memcpy(txPayload + 4,  &ay, 4);
    memcpy(txPayload + 8,  &az, 4);
    memcpy(txPayload + 12, &gx, 4);
    memcpy(txPayload + 16, &gy, 4);
    memcpy(txPayload + 20, &gz, 4);

    uint8_t txPacket[3 + 24 + 1];
    telem.buildPacket(PKT_IMU_SAMPLE, txPayload, 24, txPacket);
    telem.sendBinary(txPacket, sizeof(txPacket));

    vTaskDelayUntil(&lastWake, period);
  }
}

static void startImuStreamTask() {
  if (imuStreamTaskHandle == nullptr) {
    xTaskCreate(ImuStreamTask, "ImuStream", 2048, nullptr, 2, &imuStreamTaskHandle);
  }
}

 // ---- New: pause/resume helpers for control-side tasks ----
static void pauseControlTasks() {
  if (!controlPaused) {
    if (hPID)   vTaskSuspend(hPID);
    if (hTX)    vTaskSuspend(hTX);
    if (hSTICK) vTaskSuspend(hSTICK);
    controlPaused = true;
    Serial.println("[MODE] Paused control tasks (PID, TX, STICK).");
  }
}

static void resumeControlTasks() {
  if (controlPaused) {
    if (hPID)   vTaskResume(hPID);
    if (hTX)    vTaskResume(hTX);
    if (hSTICK) vTaskResume(hSTICK);
    controlPaused = false;
    Serial.println("[MODE] Resumed control tasks (PID, TX, STICK).");
  }
}
 
 // ---------- Telemetry TX ----------
 void telemetryTaskTX(void* pvParameters) {
   float att_temp[3]; // roll, pitch, yaw
   uint8_t payload[12];
   uint8_t packet[64];
 
   while (1) {
     xSemaphoreTake(attitudeMutex, portMAX_DELAY);
     att_temp[0] = attitude.roll;
     att_temp[1] = attitude.pitch;
     att_temp[2] = attitude.yaw;
     xSemaphoreGive(attitudeMutex);
 
     memcpy(payload + 0, &att_temp[0], 4);
     memcpy(payload + 4, &att_temp[1], 4);
     memcpy(payload + 8, &att_temp[2], 4);
 
     telem.buildPacket(0x10, (const uint8_t*)payload, 12, (uint8_t*)packet);
     telem.sendBinary((uint8_t*)packet, 3 + 12 + 1);
 
     vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz
   }
 }
 
 // ---------- Telemetry RX ----------
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
             pidPitch.setTunings(pitch_kp, pitch_ki, pitch_kd);
             pidRoll.setTunings(roll_kp, roll_ki, roll_kd);
           }
           break;
 
         case 0x21: { // Get current PID gains (6 floats = 24 bytes)
           uint8_t txPacket[28];
           float pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd;
           pidPitch.getTunings(pitch_kp, pitch_ki, pitch_kd);
           pidRoll.getTunings(roll_kp, roll_ki, roll_kd);
 
           uint8_t txPayload[24];
           memcpy(txPayload + 0,  &pitch_kp, 4);
           memcpy(txPayload + 4,  &pitch_ki, 4);
           memcpy(txPayload + 8,  &pitch_kd, 4);
           memcpy(txPayload + 12, &roll_kp,  4);
           memcpy(txPayload + 16, &roll_ki,  4);
           memcpy(txPayload + 20, &roll_kd,  4);
 
           telem.buildPacket(0x21, (const uint8_t*)txPayload, 24, (const uint8_t*)txPacket);
           telem.sendBinary((const uint8_t*)txPacket, 3 + 24 + 1);
         } break;
 
         case 0x22: // Set setpoint (legacy)
           if (len >= 8) {
             float newPitch, newRoll;
             memcpy(&newPitch, payload, 4);
             memcpy(&newRoll,  payload + 4, 4);
             xSemaphoreTake(setpointMutex, portMAX_DELAY);
             setpointPitch = newPitch;
             setpointRoll  = newRoll;
             xSemaphoreGive(setpointMutex);
           }
           break;
 
         case 0x23: { // Request current setpoint (roll, pitch) -> send (pitch, roll)
           if (len >= 8) {
             uint8_t txPayload[8];
             uint8_t txPacket[12];
             memcpy(txPayload + 0, (const void*)&setpointPitch, 4);
             memcpy(txPayload + 4, (const void*)&setpointRoll,  4);
             telem.buildPacket(0x23, (const uint8_t*)txPayload, 8, (uint8_t*)txPacket);
             telem.sendBinary((uint8_t*)txPacket, 3 + 8 + 1);
           }
         } break;
 
         case 0x24: // Set setpoint (pitch, roll)
           if (len >= 8) {
             float newPitch, newRoll;
             memcpy(&newPitch, payload, 4);
             memcpy(&newRoll,  payload + 4, 4);
             xSemaphoreTake(setpointMutex, portMAX_DELAY);
             setpointPitch = newPitch;
             setpointRoll  = newRoll;
             xSemaphoreGive(setpointMutex);
           }
           break;
 
         case 0x40: { // Stick + queries (subop in payload[0])
           if (len >= 1) {
             uint8_t subop = payload[0];
             if (subop == 0x01) {         // Enable stick mode
               stickModeEnabled = true;
               Serial.println("[RX] Stick mode: ENABLED (PID assist)");
             } else if (subop == 0x00) {  // Disable stick mode
               stickModeEnabled = false;
               Serial.println("[RX] Stick mode: DISABLED");
             } else if (subop == 0x03) {  // Throttle status query
               sendThrottleStatusReply();
             }
           }
         } break;
 
         case 0x30: { // Throttle control
           servoThrottle.attach(10);
           if (len >= 1) {
             if (payload[0] == 0x00) {
               servoThrottle.write(0);    // Kill
               gThrottleStatus = 0x00;
               Serial.println("[RX] Kill throttle: servo set to 0°");
             } else if (payload[0] == 0x01) {
               servoThrottle.write(180);  // Full
               gThrottleStatus = 0x01;
               Serial.println("[RX] Full throttle: servo set to 180°");
             }
           }
         } break;
 
         case 0x50: { // Save to EEPROM
           saveParamsToEEPROM();
           uint8_t ack[4];
           telem.buildPacket(0x50, nullptr, 0, ack);
           telem.sendBinary(ack, 4);
         } break;
 
        case 0x51: { // Reset defaults + store
           applyDefaultsAndPersist();
           uint8_t ack[4];
           telem.buildPacket(0x51, nullptr, 0, ack);
           telem.sendBinary(ack, 4);
        } break;

        case 0x11: { // IMU streaming control: payload[0] = 0x01 start, 0x00 stop (no ACK)
          if (len >= 1) {
            uint8_t cmd = payload[0];
            if (cmd == IMU_CTRL_START) {
              if (!gImuStreamEnabled) {
                pauseControlTasks();            // throttle untouched
                gImuStreamEnabled = true;
                Serial.println("[RX] IMU stream: START -> control paused; streaming ax..gz.");
              }
            } else if (cmd == IMU_CTRL_STOP) {
              if (gImuStreamEnabled) {
                gImuStreamEnabled = false;
                resumeControlTasks();
                Serial.println("[RX] IMU stream: STOP -> control resumed.");
              }
            } else {
              Serial.println("[RX] IMU stream: unknown payload (use 0x00 stop, 0x01 start).");
            }
          } else {
            Serial.println("[RX] IMU stream: missing 1-byte payload.");
          }
        } break;
        
        case 0x41: { // Mode control: 0x00 resume PID, 0x01 explicit pause (optional)
          if (len >= 1) {
            if (payload[0] == 0x00) {
              resumeControlTasks();
              Serial.println("[RX] Mode: PID control (resumed).");
            } else if (payload[0] == 0x01) {
              pauseControlTasks();
              Serial.println("[RX] Mode: paused (explicit).");
            }
          }
        } break;
        
        // New: EMERGENCY THROTTLE KILL (independent, always available)
        case 0x31: { // Safety kill (no payload)
          forceThrottleKill(); // sets servo to 0° (or 600us if you enable that line), updates gThrottleStatus
          uint8_t ack[4];
          telem.buildPacket(0x31, nullptr, 0, ack);
          telem.sendBinary(ack, 4);
          Serial.println("[RX] EMERGENCY KILL: throttle killed.");
        } break;
        

       }
     }
     vTaskDelay(pdMS_TO_TICKS(10)); // Yield
   }
 }
 
 // ---------- PID Control Task ----------
 static void PID_Control(void*) {
   servoRoll.attach(6);
   servoPitch.attach(7);
 
   pidPitch.setLimits(-90, 90);
   pidRoll.setLimits(-90, 90);
 
   Serial.println("PID_Control: starting PID control loop...");
 
   while (1) {
     Attitude curAtt;
     xSemaphoreTake(attitudeMutex, portMAX_DELAY);
     curAtt = attitude;
     xSemaphoreGive(attitudeMutex);
 
     float dt = 0.001f; // 1 ms (compute uses this; loop is 10ms delay below)
 
     float errorPitch = setpointPitch - curAtt.pitch;
     float errorRoll  = setpointRoll  - curAtt.roll;
 
     float outPitch = 0, outRoll = 0;
     if (fabs(errorPitch) > STABILITY_BAND)
       outPitch = pidPitch.compute(setpointPitch, curAtt.pitch, dt);
     if (fabs(errorRoll) > STABILITY_BAND)
       outRoll  = pidRoll.compute(setpointRoll,  curAtt.roll,  dt);
 
     float pitchCommand = constrain(90 - outPitch, 0, 180);
     float rollCommand  = constrain(90 - outRoll,  0, 180);
     servoPitch.write(pitchCommand);
     servoRoll.write(rollCommand);
 
     vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz
   }
 }
 
 // ---------- Stick Assist Task ----------
 static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
 static inline float applyDeadband(float x, float db){
   if (fabsf(x) <= db) return 0.0f;
   return (x > 0) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
 }
 static inline float us_to_norm(int us){ // 1000..2000 -> -1..+1
   float n = (us - 1500.0f) / 500.0f;
   return clampf(n, -1.0f, 1.0f);
 }
 
 static void StickAssistTask(void*) {
   ppmInit(); // from PPMcontrol.cpp
 
   float tgtPitch, tgtRoll;
   xSemaphoreTake(setpointMutex, portMAX_DELAY);
   tgtPitch = setpointPitch; tgtRoll = setpointRoll;
   xSemaphoreGive(setpointMutex);
 
   TickType_t last = xTaskGetTickCount();
 
   while (1) {
     int ch1, ch2, ch3;
     ppmRead3(ch1, ch2, ch3); // non-blocking, last valid values
 
     TickType_t now = xTaskGetTickCount();
     float dt = (now - last) / (float)configTICK_RATE_HZ;
     if (dt <= 0.0f) dt = 0.02f; // ~50 Hz fallback
     last = now;
 
     if (stickModeEnabled) {
       float rollN  = applyDeadband(us_to_norm(ch1), STICK_DEADBAND_N);
       float pitchN = applyDeadband(us_to_norm(ch2), STICK_DEADBAND_N);
 
       float desRollDeg  = clampf( rollN  * STICK_MAX_ANGLE_DEG, -STICK_MAX_ANGLE_DEG, STICK_MAX_ANGLE_DEG);
       float desPitchDeg = clampf(-pitchN * STICK_MAX_ANGLE_DEG, -STICK_MAX_ANGLE_DEG, STICK_MAX_ANGLE_DEG);
 
       float maxStep = STICK_SLEW_DPS * dt;
       float dR = clampf(desRollDeg  - tgtRoll,  -maxStep, maxStep);
       float dP = clampf(desPitchDeg - tgtPitch, -maxStep, maxStep);
       tgtRoll  += dR;
       tgtPitch += dP;
 
       xSemaphoreTake(setpointMutex, portMAX_DELAY);
       setpointRoll  = tgtRoll;
       setpointPitch = tgtPitch;
       xSemaphoreGive(setpointMutex);
 
       // If you want RC throttle passthrough in stick mode, uncomment below:
       // if (!servoThrottle.attached()) servoThrottle.attach(10);
       // int thr_us = map(constrain(ch3, 1000, 2000), 1000, 2000, 600, 2200);
       // servoThrottle.writeMicroseconds(thr_us);
     }
 
     vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz
   }
 }
 
 // ---------- Optional monitor task ----------
 void taskMonitor(void*) {
   while (1) {
     Serial.println("[Monitor] I'm alive");
     vTaskDelay(pdMS_TO_TICKS(1000));
   }
 }
 
 // ---------- Setup ----------
 FLASHMEM __attribute__((noinline)) void setup() {
   delay(2000);
   Serial.begin(115200);
   telem.init();
 
   /* --- SAFETY: throttle is ALWAYS killed at startup --- */
   forceThrottleKill();
   delay(50);
 
   if (CrashReport) {
     Serial.print(CrashReport);
     Serial.println();
     Serial.flush();
   }
 
   Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER
                       ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on "
                       __DATE__ ". ***\r\n"));
 
   imuQueue = xQueueCreate(10, sizeof(IMUPacket));
 
   attitudeMutex = xSemaphoreCreateMutex();
   if (attitudeMutex == NULL) {
     Serial.println("Failed to create attitudeMutex!");
     while (1);
   }
 
   setpointMutex = xSemaphoreCreateMutex();
   if (setpointMutex == NULL) {
     Serial.println("Failed to create setpointMutex!");
     while (1);
   }
 
  loadParamsFromEEPROM();

  
 
  xTaskCreate(IMU_read,        "IMU",    1024, nullptr, 4, &hIMU);
  xTaskCreate(PID_Control,     "PID",    2048, nullptr, 4, &hPID);
  xTaskCreate(telemetryTaskTX, "TX",     1024, nullptr, 4, &hTX);
  xTaskCreate(telemetryTaskRX, "RX",     1024, nullptr, 4, &hRX);
  xTaskCreate(StickAssistTask, "STICK",  1024, nullptr, 4, &hSTICK);
  //  xTaskCreate(IMU_read,        "IMU",    1024, nullptr, 4, nullptr);
  //  xTaskCreate(PID_Control,     "PID",    2048, nullptr, 4, nullptr);
  //  xTaskCreate(telemetryTaskTX, "TX",     1024, nullptr, 4, nullptr);
  //  xTaskCreate(telemetryTaskRX, "RX",     1024, nullptr, 4, nullptr);
  //  xTaskCreate(StickAssistTask, "STICK",  1024, nullptr, 4, nullptr);
   // xTaskCreate(taskMonitor,   "Monitor", 1024, nullptr, 1, nullptr);
 
  startImuStreamTask();
  Serial.println("setup(): starting scheduler...");
  Serial.flush();
 
  vTaskStartScheduler();
 }
 
 // ---------- Loop ----------
 void loop() {}
 
/**
 * @file    main.cpp
 * @brief   FreeRTOS example for Teensy boards
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

 #include "arduino_freertos.h"
 #include "avr/pgmspace.h"
 #include "Kalman.h"
 #include "IMU.h"

 IMU imu;

 
 
 static void IMU_read(void*) {

    imu.begin();

     
 }
 
//  static void task2(void*) {
//      Serial.begin(9600);
//      while (true) {
//          Serial.println("TICK");
//          vTaskDelay(pdMS_TO_TICKS(1'000));
 
//          Serial.println("TOCK");
//          vTaskDelay(pdMS_TO_TICKS(1'000));
//      }
//  }
 
 FLASHMEM __attribute__((noinline)) void setup() {

     delay(2000);
     Serial.begin(0);
     //delay(2'000);
 
     if (CrashReport) {
         Serial.print(CrashReport);
         Serial.println();
         Serial.flush();
     }
 
     Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));
 
     xTaskCreate(IMU_read, "task1", 128, nullptr, 2, nullptr);
     //xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);
 
     Serial.println("setup(): starting scheduler...");
     Serial.flush();
 
     vTaskStartScheduler();
 }
 
 void loop() {}
 
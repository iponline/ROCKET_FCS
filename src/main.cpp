#include <Arduino.h>
#include "TeensyThreads.h"
#include <avr/io.h>


const int LED = 13;

volatile int blinkcode = 0;

void blinkthread() {
  while(1) {
    if (blinkcode) {
      for (int i=0; i<blinkcode; i++) {
        digitalWrite(LED, HIGH);
        threads.delay(500);
        digitalWrite(LED, LOW);
        threads.delay(500);
      }
      blinkcode = 0;
    }
    threads.yield();
  }
}

void setup() {
  //delay(1000);
  pinMode(LED, OUTPUT);
  threads.addThread(blinkthread);
}

int count = 0;

void loop() {
  count++;
  blinkcode = count;
  //delay(5000);
  Serial.println(count);
}
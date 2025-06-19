#include "Telemetry.h"

#define TELEMETRY_SERIAL Serial1
#define TELEMETRY_BAUD   57600

void Telemetry::init() {

  TELEMETRY_SERIAL.begin(TELEMETRY_BAUD);
  delay(2000);

  enterCommandMode();

  // 2->2400, 4->4800, 8->9600, 16->19200, 32->38400, 64->76800, 128->125000  
  setAirDataRate(64); 
  sendATCommand("AT&W");
  sendATCommand("ATZ");

}

void Telemetry::enterCommandMode() {

  delay(1000);
  TELEMETRY_SERIAL.print("+++");
  delay(1000);

  unsigned long timeout = millis() + 1000;
  while (millis() < timeout) {
    while (TELEMETRY_SERIAL.available()) {
      Serial.write(TELEMETRY_SERIAL.read());
    }
  }
}

void Telemetry::sendATCommand(const char* cmd) {
  Serial.print("Sending AT command: ");
  Serial.println(cmd);

  TELEMETRY_SERIAL.print(cmd);
  TELEMETRY_SERIAL.print("\r\n");

  delay(200);
  unsigned long timeout = millis() + 1000;
  while (millis() < timeout) {
    while (TELEMETRY_SERIAL.available()) {
      Serial.write(TELEMETRY_SERIAL.read());
    }
  }
}

void Telemetry::setAirDataRate(uint8_t rate) {
  char cmd[16];
  sprintf(cmd, "ATS3=%d", rate);
  sendATCommand(cmd);
}

void Telemetry::sendMessage(const char* msg) {
  TELEMETRY_SERIAL.println(msg);  // Send string with newline
}

void Telemetry::sendBinary(const uint8_t* data, size_t len) {
  TELEMETRY_SERIAL.write(data, len); // Send raw binary buffer
}

bool Telemetry::receiveMessage(char* buffer, size_t maxLen) {
  size_t idx = 0;
  unsigned long timeout = millis() + 100;

  while (millis() < timeout && idx < maxLen - 1) {
    if (TELEMETRY_SERIAL.available()) {
      char c = TELEMETRY_SERIAL.read();
      if (c == '\n') {
        break;
      }
      buffer[idx++] = c;
    }
  }

  buffer[idx] = '\0';
  return (idx > 0);
}

size_t Telemetry::receiveBinary(uint8_t* buffer, size_t maxLen) {
  size_t count = 0;
  unsigned long timeout = millis() + 10;

  while (millis() < timeout && count < maxLen) {
    if (TELEMETRY_SERIAL.available()) {
      buffer[count++] = TELEMETRY_SERIAL.read();
    }
  }

  return count;
}

void Telemetry::setBaudRate(uint32_t baud) {


    uint8_t baudCode = 0;
  
    // SiK ATS2 codes:
    // 1=1200, 2=2400, 3=4800, 4=9600, 5=19200, 6=38400, 7=57600, 8=115200
    switch (baud) {
      case 1200:   baudCode = 1; break;
      case 2400:   baudCode = 2; break;
      case 4800:   baudCode = 3; break;
      case 9600:   baudCode = 4; break;
      case 19200:  baudCode = 5; break;
      case 38400:  baudCode = 6; break;
      case 57600:  baudCode = 7; break;
      case 115200: baudCode = 8; break;

      default:
        Serial.println("Unsupported baud rate");
        return;
    }
  
    char cmd[16];
    sprintf(cmd, "ATS2=%d", baudCode);
    sendATCommand(cmd);

  }

  void Telemetry::buildPacket(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* outBuffer) {

    const uint8_t START_BYTE = 0xAA;
  
    outBuffer[0] = START_BYTE; // Start
    outBuffer[1] = type;       // Packet type
    outBuffer[2] = len;        // Payload length
  
    // Copy payload
    for (uint8_t i = 0; i < len; i++) {
      outBuffer[3 + i] = payload[i];
    }
  
    // Calculate checksum: XOR of all previous bytes
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 3 + len; i++) {
      checksum ^= outBuffer[i];
    }
  
    outBuffer[3 + len] = checksum; // Append checksum

    // === Debug Print ===
    // Serial.print("Telemetry Packet: ");
    // for (uint8_t i = 0; i < 3 + len + 1; i++) {  // 3 header + payload + checksum
    //      if (outBuffer[i] < 0x10) Serial.print("0"); // pad leading zero
    //      Serial.print(outBuffer[i], HEX);
    //      Serial.print(" ");
    // }
    // Serial.println();

  }

  bool Telemetry::receivePacket(uint8_t* type, uint8_t* payload, uint8_t* length) {
    
    const uint8_t START_BYTE = 0xAA;
    static enum { WAIT_START, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_START;
    static uint8_t buffer[260];  // Max 256 + header + checksum
    static uint8_t index = 0;
    static uint8_t expectedLength = 0;
  
    while (TELEMETRY_SERIAL.available()) {
      uint8_t byteIn = TELEMETRY_SERIAL.read();
  
      switch (state) {
        case WAIT_START:
          if (byteIn == START_BYTE) {
            buffer[0] = byteIn;
            index = 1;
            state = WAIT_TYPE;
          }
          break;
  
        case WAIT_TYPE:
          buffer[index++] = byteIn;
          *type = byteIn;
          state = WAIT_LEN;
          break;
  
        case WAIT_LEN:
          buffer[index++] = byteIn;
          expectedLength = byteIn;
          *length = byteIn;
  
          if (expectedLength == 0)
            state = WAIT_CHECKSUM;
          else
            state = WAIT_PAYLOAD;
          break;
  
        case WAIT_PAYLOAD:
          buffer[index++] = byteIn;
          if (index == 3 + expectedLength)
            state = WAIT_CHECKSUM;
          break;
  
        case WAIT_CHECKSUM:
          buffer[index++] = byteIn;
  
          // Calculate checksum
          uint8_t checksum = 0;
          for (uint8_t i = 0; i < index - 1; i++) {
            checksum ^= buffer[i];
          }
  
          if (checksum == byteIn) {
            memcpy(payload, &buffer[3], expectedLength);  // Extract payload
            state = WAIT_START;
            return true; // Valid packet received
          } else {
            Serial.println("Invalid checksum!");
          }
  
          // Reset on error
          state = WAIT_START;
          break;
      }
    }
  
    return false; // No valid packet yet
  }
  
  

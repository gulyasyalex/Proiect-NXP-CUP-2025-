#include <Arduino.h>
#define TFMINI_SERIAL Serial2  // Use UART2 (Pin 7 RX, Pin 8 TX)

void setup() {
    Serial.begin(115200);        // USB Serial for debugging
    TFMINI_SERIAL.begin(115200); // TFmini Plus runs at 115200 baud
}

void loop() {
    if (TFMINI_SERIAL.available() >= 9) { // Ensure we have a full packet
        uint8_t buf[9];
        
        // Read data from TFmini
        TFMINI_SERIAL.readBytes(buf, 9);

        // Verify the packet header
        if (buf[0] == 0x59 && buf[1] == 0x59) {
            uint16_t distance = buf[2] | (buf[3] << 8);  // Distance in cm
            uint16_t strength = buf[4] | (buf[5] << 8);  // Signal strength
            uint8_t checksum = buf[8];

            // Validate checksum
            uint8_t sum = 0;
            for (int i = 0; i < 8; i++) {
                sum += buf[i];
            }

            if (sum == checksum) {
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.print(" cm, Strength: ");
                Serial.println(strength);
            } else {
                Serial.println("Checksum error!");
            }
        }
    }
}

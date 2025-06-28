#include <Arduino.h>
#include <PWMServo.h>

PWMServo ESC;
#define DRIVER_MOTOR_PIN 9
#define STANDSTILL_SPEED 91.0f
#define MAX_MOTOR_SPEED 121.0f

#define RPM_SENSOR_PIN 14
#define PULSES_PER_REV 16
#define DISTANCE_PER_REVOLUTION 7 // cm
#define SPEED_SAMPLE_SIZE 10

volatile unsigned long lastPulseMicros = 0;
volatile unsigned long periodBuffer[SPEED_SAMPLE_SIZE];
volatile uint8_t bufferIndex = 0;

unsigned long lastStepMillis = 0;
int escTestValue = 90;
bool autoTestDone = false;

void pulseISR() {
  unsigned long now = micros();
  unsigned long period = now - lastPulseMicros;
  lastPulseMicros = now;
  if (period < 100) return;
  periodBuffer[bufferIndex] = period;
  bufferIndex = (bufferIndex + 1) % SPEED_SAMPLE_SIZE;
}

unsigned long getMedian(unsigned long* arr, int size) {
  unsigned long temp[size];
  memcpy(temp, arr, sizeof(unsigned long) * size);
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        unsigned long t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return (size % 2) ? temp[size / 2] : (temp[size / 2 - 1] + temp[size / 2]) / 2;
}

void setup() {
  ESC.attach(DRIVER_MOTOR_PIN, 1000, 2000);
  Serial.begin(230400);

  pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), pulseISR, FALLING);

  Serial.begin(230400);
   // === Step 1: Set Throttle to Maximum ===
  Serial.println("Setting throttle to MAX.");
  // Write 180 degrees to produce the maximum pulse width.
  ESC.write(MAX_MOTOR_SPEED);
  delay(1000);  // Hold the maximum throttle signal for 1 second.

  // === Step 2: Power On the ESC ===
  // Now, power on your ESC (connect the battery) while the throttle is still set at maximum.
  // Many ESCs use beeps to confirm they have recognized the maximum throttle.
  Serial.println("Now, power on the ESC (if not already powered).");
  delay(3000);  // Wait 3 seconds (adjust as needed based on your ESC's beep sequence).

  // === Step 3: Lower Throttle to Minimum ===
  Serial.println("Setting throttle to MIN.");
  // Write 0 degrees to produce the minimum pulse width.
  ESC.write(STANDSTILL_SPEED - 1);
  delay(1000);  // Hold this setting for 1 second to complete calibration.

  Serial.println("Calibration complete. ESC should now be calibrated. Start Test!");
  lastStepMillis = millis();
}

void loop() {
  if (autoTestDone) return;

  // Run every 1 second
  if (millis() - lastStepMillis >= 1000) {
    lastStepMillis = millis();

    ESC.write(escTestValue);

    static unsigned long bufferCopy[SPEED_SAMPLE_SIZE];
    noInterrupts();
    memcpy(bufferCopy, periodBuffer, sizeof(bufferCopy));
    unsigned long lastPulse = lastPulseMicros;
    interrupts();

    unsigned long median = getMedian(bufferCopy, SPEED_SAMPLE_SIZE);
    if (micros() - lastPulse > 500000) median = 0;

    float rps = (median > 0) ? (1.0e6 / (median * PULSES_PER_REV)) : 0;
    float speed = rps * DISTANCE_PER_REVOLUTION;

    Serial.print("ESC: ");
    Serial.print(escTestValue);
    Serial.print(" -> Speed: ");
    Serial.print(speed, 2);
    Serial.println(" cm/s");

    escTestValue++;
    if (escTestValue > 120) {
      ESC.write(STANDSTILL_SPEED);
      autoTestDone = true;
      Serial.println("Auto test complete.");
    }
  }
}

#include <Arduino.h>
#include <PWMServo.h>
#include "SteeringWheel.h"
#include <IntervalTimer.h>

#define BAUD_RATE  230400
#define RPM_SENSOR_PIN  14
#define PULSES_PER_REV  8
#define RPM_MEDIAN_FILTER_SAMPLE_SIZE  5
#define DISTANCE_PER_REVOLUTION 0.1                                             // Revolution = 10 cm on the ground

volatile unsigned long RpsLastMicros = 0;                                          // Time of the last pulse
volatile double pulseCounter = 0;                                               // Current index in buffer

void pulseISR();
unsigned long getMedian(unsigned long *array, int size);

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), pulseISR, RISING);

  pulseCounter = 0;
}

void loop() {

}

void pulseISR() {
  unsigned long currentMicros = micros();
  unsigned long newPeriod = currentMicros - RpsLastMicros;
  RpsLastMicros = currentMicros;
  if (newPeriod < 100) {  // 100 µs is arbitrary; tweak as needed
    return; 
  }
  pulseCounter++;
  Serial.print("PulseCounter: ");
  Serial.println(pulseCounter);
}


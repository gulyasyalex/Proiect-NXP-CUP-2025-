#include <Arduino.h>
#include <PWMServo.h>
#include "SteeringWheel.h"
#include <IntervalTimer.h>

#define BAUD_RATE  230400
#define RPM_SENSOR_PIN  10
#define PULSES_PER_REV  8
#define RPM_MEDIAN_FILTER_SAMPLE_SIZE  5
#define DISTANCE_PER_REVOLUTION 0.1                                             // Revolution = 10 cm on the ground

volatile unsigned long RpsLastMicros = 0;                                          // Time of the last pulse
volatile unsigned long RpsPeriodBuffer[RPM_MEDIAN_FILTER_SAMPLE_SIZE];             // Circular buffer
volatile uint8_t RpsBufferIndex = 0;                                               // Current index in buffer

void pulseISR();
unsigned long getMedian(unsigned long *array, int size);

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), pulseISR, RISING);

  Serial.println("Starting...");

  // Initialize the period buffer with a default (avoid zero)
  for (int i = 0; i < RPM_MEDIAN_FILTER_SAMPLE_SIZE; i++) {
    RpsPeriodBuffer[i] = 100000; 
  }
}

void loop() {
  static unsigned long RpsLocalBuffer[RPM_MEDIAN_FILTER_SAMPLE_SIZE];

  noInterrupts();
  for (int i = 0; i < RPM_MEDIAN_FILTER_SAMPLE_SIZE; i++) {
    RpsLocalBuffer[i] = RpsPeriodBuffer[i];
  }
  interrupts();

  unsigned long medianPeriod = getMedian(RpsLocalBuffer, RPM_MEDIAN_FILTER_SAMPLE_SIZE);

  float rps = 0.0; 
  if (medianPeriod > 0) {
    // RPS = 1,000,000 us/second / (medianPeriod * pulsesPerRevolution)
    rps = (1.0e6) / ( (float)medianPeriod * (float)PULSES_PER_REV );
  }

  float speed_m_s = DISTANCE_PER_REVOLUTION * rps;

  // Serial.print("  ->  Speed (m/s): ");
  Serial.println(speed_m_s);
}

void pulseISR() {
  unsigned long currentMicros = micros();
  unsigned long newPeriod = currentMicros - RpsLastMicros;
  RpsLastMicros = currentMicros;
  if (newPeriod < 100) {  // 100 µs is arbitrary; tweak as needed
    return; 
  }
  RpsPeriodBuffer[RpsBufferIndex] = newPeriod;
  RpsBufferIndex = (RpsBufferIndex + 1) % RPM_MEDIAN_FILTER_SAMPLE_SIZE;
}

unsigned long getMedian(unsigned long *array, int size) {
  // Simple bubble sort
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (array[j] < array[i]) {
        unsigned long temp = array[i];
        array[i] = array[j];
        array[j] = temp;
      }
    }
  }
  
  // Return middle element
  if (size % 2 == 1) {
    return array[size / 2];
  } else {
    int mid = size / 2;
    return (array[mid - 1] + array[mid]) / 2;
  }
}

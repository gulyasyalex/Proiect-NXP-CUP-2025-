#include <Arduino.h>
#include <PWMServo.h>
#include "SteeringWheel.h"
#include <IntervalTimer.h>

#define BAUD_RATE  230400
// Calibration coefficients (from your MATLAB fit)
const float A = -0.02545;
const float B = 0.929;
const float C = 17.48;

// Desired speed in cm/s (adjust as needed)
const float desiredSpeed = 500.0;

// f(t) = A*t^3 + B*t^2 + C*t - v
float f(float t, float v) {
  return A * t * t * t + B * t * t + C * t - v;
}

// f'(t) = 3*A*t^2 + 2*B*t + C
float f_prime(float t) {
  return 3.0 * A * t * t + 2.0 * B * t + C;
}

// Newton-Raphson method to solve f(t) = 0 for t
float solveForT(float v, float initial_guess = 10.0) {
  float t = initial_guess;
  const int max_iter = 100;
  const float tol = 1e-6;

  for (int i = 0; i < max_iter; ++i) {
    float f_val = f(t, v);
    float df_val = f_prime(t);

    if (fabs(df_val) < 1e-9) { // Avoid division by near-zero
      Serial.println("Derivative too small; stopping iteration.");
      break;
    }

    float t_next = t - f_val / df_val;
    if (fabs(t_next - t) < tol) {
      t = t_next;
      break;
    }
    t = t_next;
  }
  return t;
}

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) { } // Wait for serial port to connect (if needed)

  // Solve for t using Newton-Raphson
  float t_solution = solveForT(desiredSpeed);

  // Convert t back to servo write value (servo value = t + 91)
  int servoValue = t_solution + 91;

  Serial.print("For a desired speed of ");
  Serial.print(desiredSpeed);
  Serial.print(" cm/s, the computed servo value is: ");
  Serial.println(servoValue);
}

void loop() {
  // Nothing to do in loop()
}

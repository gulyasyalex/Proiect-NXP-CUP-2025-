/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Arduino.h>
#include <PWMServo.h>

PWMServo edfFan;     // create servo object to control the ESC
#define EDF_FAN_PIN  5
#define STANDSTILL_SPEED 91.0f
#define MAX_MOTOR_SPEED 180.0f
int num = 0;

void setup() 
{
  // Attach the  ESC on pin 9
  edfFan.attach(EDF_FAN_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  Serial.begin(230400);
   // === Step 1: Set Throttle to Maximum ===
  Serial.println("Setting throttle to MAX.");
  // Write 180 degrees to produce the maximum pulse width.
  edfFan.write(MAX_MOTOR_SPEED);
  delay(1000);  // Hold the maximum throttle signal for 1 second.

  // === Step 2: Power On the ESC ===
  // Now, power on your ESC (connect the battery) while the throttle is still set at maximum.
  // Many ESCs use beeps to confirm they have recognized the maximum throttle.
  Serial.println("Now, power on the ESC (if not already powered).");
  delay(3000);  // Wait 3 seconds (adjust as needed based on your ESC's beep sequence).

  // === Step 3: Lower Throttle to Minimum ===
  Serial.println("Setting throttle to MIN.");
  // Write 0 degrees to produce the minimum pulse width.
  edfFan.write(STANDSTILL_SPEED - 1);
  delay(1000);  // Hold this setting for 1 second to complete calibration.

  Serial.println("Calibration complete. ESC should now be calibrated. Start Test!");
}

void loop() 
{
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n'); // read until newline
    int num = inputString.toInt();
    Serial.print("Min Value(91)/Max value(121):");
    Serial.println(num);
    edfFan.write(num);
  }

}
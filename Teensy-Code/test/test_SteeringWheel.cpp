#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
// #include <stdio.h>
// #include <stdint.h>

#define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  60    // 30 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   120   // 120 max left

#define STEERING_SERVO_PIN  3


float servoAngle = 0;
SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);

void parseSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read input until newline character
        input.trim(); // Remove any trailing or leading whitespaces
        servoAngle = input.toInt();

        Serial.print("Parsed Values -> Servo Angle: ");
        Serial.print(servoAngle);
    } else {
        Serial.println("Enter input in the format: servoAngle");
    }
    
}
// 90;10;2
//servoAngle;servoTimeout;servoAngleIncrease

void setup() {  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(230400);
    while (!Serial);
    Serial.println("Starting...");
    steeringWheel.attach(STEERING_SERVO_PIN);   
    steeringWheel.setSteeringAngleDeg(0); 
}

void loop() {
    parseSerialInput();
    steeringWheel.setSteeringAngleDeg(servoAngle);
    Serial.println(servoAngle);
    delay(150);
    /*for (int servoAngle = -30; servoAngle <= 30; servoAngle++)
    {
        steeringWheel.setSteeringAngleDeg(servoAngle);
        Serial.println(servoAngle);
        delay(50);
    }
    for (int servoAngle = 30; servoAngle >= -30; servoAngle--)
    {
        steeringWheel.setSteeringAngleDeg(servoAngle);
        Serial.println(servoAngle);
        delay(50);
    }*/
}

/*
//Working example values range from 30-120

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
// #include <stdio.h>
// #include <stdint.h>

#define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  30    // 30 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   120   // 120 max left

#define STEERING_SERVO_PIN  3


float servoAngle = 90;
int servoTimeout = 7;
int servoAngleIncrease = 4;
SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);

void parseSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read input until newline character
        input.trim(); // Remove any trailing or leading whitespaces

        // Split the input based on semicolons
        int firstSeparator = input.indexOf(';');
        int secondSeparator = input.indexOf(';', firstSeparator + 1);

        if (firstSeparator != -1 && secondSeparator != -1) {
            servoAngle = input.substring(0, firstSeparator).toInt();
            servoTimeout = input.substring(firstSeparator + 1, secondSeparator).toInt();
            servoAngleIncrease = input.substring(secondSeparator + 1).toInt();

            Serial.print("Parsed Values -> Servo Angle: ");
            Serial.print(servoAngle);
            Serial.print(", Servo Timeout: ");
            Serial.print(servoTimeout);
            Serial.print(", Servo Angle Increase: ");
            Serial.println(servoAngleIncrease);
        } else {
            Serial.println("Enter input in the format: servoAngle;servoTimeout;servoAngleIncrease");
        }
    }
}
// 90;10;2
//servoAngle;servoTimeout;servoAngleIncrease

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Starting...");
    steeringWheel.attach(STEERING_SERVO_PIN);    
}

void loop() {
    //parseSerialInput();
    for (int servoAngle = 30; servoAngle <= 120; servoAngle++)
    {
        steeringWheel.SlowWrite(servoAngle, servoTimeout, servoAngleIncrease);
        //steeringWheel.write(servoAngle);
        Serial.println("servoTempAngle: " + String(steeringWheel.getTempAngle()) + "\tservoFinalAngle: " + String(steeringWheel.getFinalAngle()) + "\tsteerANgle: " + String(steeringWheel.getSteeringAngle()));
        Serial.println(servoAngle);
        delay(50);
    }
    for (int servoAngle = 120; servoAngle >= 30; servoAngle--)
    {
        steeringWheel.SlowWrite(servoAngle, servoTimeout, servoAngleIncrease);
        //steeringWheel.write(servoAngle);
        Serial.println("servoTempAngle: " + String(steeringWheel.getTempAngle()) + "\tservoFinalAngle: " + String(steeringWheel.getFinalAngle()) + "\tsteerANgle: " + String(steeringWheel.getSteeringAngle()));
        Serial.println(servoAngle);
        delay(50);
    }
}

*/
float pulse_width;

#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  pinMode(4, INPUT);
  Serial.println("Setup done");
}

void loop()
{
  pulse_width = pulseIn(4, HIGH);
  if(pulse_width != 0){
        pulse_width = pulse_width/10.0; // 10usec = 1 cm of distance for LIDAR-Lite
  	Serial.println(pulse_width);
  }
  delay(200);
}
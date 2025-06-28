/*#include <Arduino.h>

int pw_pin = 4; // Pin connected to the PWM output of the MaxSonar sensor
long pulse_width; // Variable to store pulse width in microseconds
float distance_cm; // Variable to store the calculated distance in cm

void setup() {
    pinMode(pw_pin, INPUT);  // Set the pin as input to read the PWM signal
    Serial.begin(230400);    // Start serial communication at 230400 baud rate
}

void loop() {
    // Measure the duration of the PWM pulse (high state)
    pulse_width = pulseIn(pw_pin, HIGH);

    // Calculate distance in cm (1 cm = 58 µs)
    distance_cm = pulse_width / 58.0;

    // Print the distance to the serial monitor
    Serial.print("The distance = ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    delay(100); // Optional delay for readability
}
*/
#include <Arduino.h>

int pwmPin = 4;                
long pulse_width;              
float distance_cm;             
const int bufferSize = 5;           // Size of the buffer for median filtering
float distanceBuffer[bufferSize];   
int bufferIndex = 0;                // Index to keep track of the current position in the buffer

void setup() {
    pinMode(pwmPin, INPUT);    
    Serial.begin(230400);    

    for (int i = 0; i < bufferSize; i++) {
        distanceBuffer[i] = 0;
    }
}

void addToBuffer(float value) {
    distanceBuffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % bufferSize; 
}

float calculateMedian(float *buffer, int size) {
    float sorted[size];                                     // temporary array for sorting
    memcpy(sorted, buffer, sizeof(float) * size); 

    // Bubble sort
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    if (size % 2 == 0) {
        // Even number of elements: return the average of the two middle values
        return (sorted[size / 2 - 1] + sorted[size / 2]) / 2.0;
    } else {
        // Odd number of elements: return the middle value
        return sorted[size / 2];
    }
}

void loop() {
    pulse_width = pulseIn(pwmPin, HIGH);

    // Calculate distance in cm (1 cm = 58 µs)
    distance_cm = pulse_width / 58.0;

    addToBuffer(distance_cm);

    float medianDistance = calculateMedian(distanceBuffer, bufferSize);

    Serial.print("The distance (median) = ");
    Serial.print(medianDistance);
    Serial.println(" cm");

}

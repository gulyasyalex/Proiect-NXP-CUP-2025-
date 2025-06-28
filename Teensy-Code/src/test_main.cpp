#include <Arduino.h>
#include <PWMServo.h>
#include "SteeringWheel.h"
#include <IntervalTimer.h>
#include "TFMiniPlus.h"

TFMiniPlus tfmini;

#define BAUD_RATE  230400

#define PID_SPEED_CALIBRATION  0
   
// ================== Lidar / RADAR ==================
#define TFMINI_SERIAL Serial2                                                 // Use UART2 (Pin 7 RX, Pin 8 TX)
#define TFMINI_BAUD_RATE 115200  

#define RADAR_MOTOR_PIN  4

#define RADAR_SERVO_ANGLE_MIDDLE     70                                       // 70 middle
#define RADAR_SERVO_ANGLE_MAX_RIGHT  60                                       // 50 max right
#define RADAR_SERVO_ANGLE_MAX_LEFT   80                                       // 90 max left


// ================== Steering Servo ==================
#define STEERING_SERVO_PIN  3

#define STEERING_SERVO_ANGLE_MIDDLE     90                                    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  60                                    // 30 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   120                                   // 120 max left

#define MAX_LIDAR_DISTANCE   500                                 

// ================== DRIVER_MOTOR ==================
#define DRIVER_MOTOR_PIN  9

#define STANDSTILL_SPEED 90.0f
#define MAX_MOTOR_SPEED 121.0f

#define FINISH_LINE_SPEED 60                                                       // cm/s
#define MAX_DESIRED_SPEED 350//215                                                 // cm/s

// ================== EDF_FAN ==================
#define EDF_FAN_PIN  5

// ================== RPM Sensor ==================
#define RPM_SENSOR_PIN  14

#define PULSES_PER_REV  16
#define RPM_MEDIAN_FILTER_SAMPLE_SIZE  10
#define DISTANCE_PER_REVOLUTION 7                                              // Revolution = 7 cm on the ground

// ================== Emergency Remote ==================
#define REMOTE_START_PIN 15
#define REMOTE_STOP_PIN 16

// ================== Status LED pin ==================
#define STATUS_LED_BLUE_PIN 20
#define STATUS_LED_YELLOW_PIN 21


SteeringWheel servoSteering(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
SteeringWheel radarServo(RADAR_SERVO_ANGLE_MAX_LEFT, RADAR_SERVO_ANGLE_MIDDLE, RADAR_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotor;  
PWMServo edfFan;  


volatile bool enableCarEngine = false;
volatile bool isFinishLineDetected = false;
volatile bool remoteControlOverride = false;

// Status LED
bool isBlueStatusLedOn = false;
bool isYellowStatusLedOn = false;

bool isRaceDone = false;

double lastServoAngle = 0.0;
double lastRadarAngle = 0.0;

// Driving Speed Variables
double desiredSpeed = 0;
double desiredSpeedEDF = 0;
double maxDesiredSpeed = MAX_DESIRED_SPEED;                 
//double driverMotorSpeed = STANDSTILL_SPEED;

// Radar Variables
int distanceFromLidarInCm = 10000;                                              // Something over the minimal threshold sent over Serial
uint16_t distanceBeforeIssuesAppear;
bool isRadarEnabled = false;


// Timing Variables
unsigned long previousMillisSteering = 0;
const unsigned long updateIntervalSteering = 20;                                // 20 ms(50hz) interval
unsigned long previousMillisRadar = 0;
const unsigned long updateIntervalRadar = 100;                                  // 100 ms(10hz) interval

int checksumCalc = 0;
String checksumStr;

// Timing variables for button debounce
unsigned long lastIncreaseMillis = 0;
unsigned long lastDecreaseMillis = 0;
const unsigned long debounceDelay = 200;                                        // 200 ms debounce delay

// Timing Variables for RPS
volatile unsigned long RpsLastMicros = 0;                                       // Time of the last pulse
volatile unsigned long RpsPeriodBuffer[RPM_MEDIAN_FILTER_SAMPLE_SIZE];          // Circular buffer
volatile uint8_t RpsBufferIndex = 0;                                            // Current index in buffer
// PID
double error;
double previousError = 0.0;
double integral = 0.0;
unsigned long lastPIDUpdate = 0;
const unsigned long PID_UPDATE_INTERVAL = 10;                                   // 10ms(100hz) interval for PID updates

#define Kp  0.0015                                                              // INFO: For smaller gear:0.001
#define Ki  0.0001                                                              // INFO: For smaller gear:0.0
#define Kd  0.00001                                                             // INFO: For smaller gear:0.001
#define RPS_TIMEOUT 500000                                                      // 500ms Timeout
float kp = Kp;
float ki = Ki;
float kd = Kd;

IntervalTimer lidarTimer;


void setLedStatus(int ledPin, bool isStatusLedOn);
void parseSerialInput();
void processSerialCommand(const char* command);
void emergencyStopISR();
void emergencyStartISR();
void pulseISR();
unsigned long getMedian(unsigned long *array, int size);
void updateSpeedPID(float actualSpeed, float desiredSpeed);
float speedToServoValue(float speed);
int getDistanceTfMiniPlus();
void readLidar();
int calculateRadarServoAngle(int lastServoAngle);
#if (1 == PID_SPEED_CALIBRATION)    
  void processSerialCommandPidCalibration(const char* command);
#endif

// Buffer for incoming serial data
const uint8_t SERIAL_BUFFER_SIZE = 128;
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialBufferIndex = 0;


void setup() {
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(STATUS_LED_BLUE_PIN, OUTPUT);
    pinMode(STATUS_LED_YELLOW_PIN, OUTPUT);
    servoSteering.attach(STEERING_SERVO_PIN);   
    servoSteering.setSteeringAngleDeg(lastServoAngle); 
    
    radarServo.attach(RADAR_MOTOR_PIN);   
    radarServo.setSteeringAngleDeg(lastRadarAngle); 

    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
    pinMode(REMOTE_STOP_PIN, INPUT_PULLDOWN);
    pinMode(REMOTE_START_PIN, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), pulseISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(REMOTE_STOP_PIN), emergencyStopISR, RISING);
    attachInterrupt(digitalPinToInterrupt(REMOTE_START_PIN), emergencyStartISR, RISING);
    
    Serial.begin(BAUD_RATE);
    TFMINI_SERIAL.begin(TFMINI_BAUD_RATE);
    while(!Serial){
      delay(10);
    }
    while(!TFMINI_SERIAL){
      delay(10);
    }
    tfmini.begin(&TFMINI_SERIAL);

    driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000); 
    edfFan.attach(EDF_FAN_PIN, 1000, 2000); 
    
    edfFan.write(STANDSTILL_SPEED);
    delay(7000);
}
void loop() {

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  parseSerialInput();

  setLedStatus(STATUS_LED_BLUE_PIN, isBlueStatusLedOn);
  setLedStatus(STATUS_LED_YELLOW_PIN, isYellowStatusLedOn);

  static unsigned long RpsLocalBuffer[RPM_MEDIAN_FILTER_SAMPLE_SIZE];

  noInterrupts();
  for (int i = 0; i < RPM_MEDIAN_FILTER_SAMPLE_SIZE; i++) {
    RpsLocalBuffer[i] = RpsPeriodBuffer[i];
  }
  unsigned long lastPulseTime = RpsLastMicros;
  interrupts();

  unsigned long medianPeriod = getMedian(RpsLocalBuffer, RPM_MEDIAN_FILTER_SAMPLE_SIZE);

  if (micros() - lastPulseTime > RPS_TIMEOUT){
      medianPeriod = 0;
  }
  float rps = 0.0; 
  if (medianPeriod > 0) {
      // RPS = 1,000,000 us/second / (medianPeriod * pulsesPerRevolution)
      rps = (1.0e6) / ( (float)medianPeriod * (float)PULSES_PER_REV );
  }

  float actualSpeed = DISTANCE_PER_REVOLUTION * rps;

  int distance = getDistanceTfMiniPlus();
  if (distance > 0) {
    if(distance > MAX_LIDAR_DISTANCE){
      distanceFromLidarInCm = MAX_LIDAR_DISTANCE;
    }else{
      distanceFromLidarInCm = distance;
    }
  }
  
  if(isFinishLineDetected)
  {
    if(isRadarEnabled){
      lastRadarAngle = calculateRadarServoAngle(lastRadarAngle);
    }
    else
    {
      lastRadarAngle = 0;
    }
    
    unsigned long currentMillisRadar = millis();

    if (currentMillisRadar - previousMillisRadar >= updateIntervalRadar)
    {
        previousMillisRadar = currentMillisRadar;

        radarServo.setSteeringAngleDeg(lastRadarAngle);
    }
  }

  float writeSpeedDriverMotor = speedToServoValue(desiredSpeed);
  float writeSpeedEDF = speedToServoValue(desiredSpeedEDF);
  
  if (writeSpeedDriverMotor >= 0) {   
    driverMotor.write(writeSpeedDriverMotor);
  }
  if (writeSpeedEDF >= 0) {   
    edfFan.write(writeSpeedEDF);
  }
 
  Serial.print(desiredSpeed);
  Serial.print(";");
  Serial.println(distanceFromLidarInCm);

  unsigned long currentMillisSteering = millis();

  if (currentMillisSteering - previousMillisSteering >= updateIntervalSteering) {
      previousMillisSteering = currentMillisSteering;

      servoSteering.setSteeringAngleDeg(lastServoAngle);
  }
  delay(1);
}


void setLedStatus(int ledPin, bool isStatusLedOn)
{
  
  if(isStatusLedOn)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
}

void parseSerialInput() {

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      serialBuffer[serialBufferIndex] = '\0';
      #if (1 == PID_SPEED_CALIBRATION)
        processSerialCommandPidCalibration(serialBuffer);
      #else
        processSerialCommand(serialBuffer);
      #endif
      // Reset buffer index for the next command
      serialBufferIndex = 0;
    }
    else {
      if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[serialBufferIndex++] = incomingChar;
      }
    }
  }
}

// Parse the command received via serial input.
// Expected format: "<enableCarEngine>;<servoAngle>;<speed>;<isFinishLineDetected>;
//<distanceBeforeIssuesAppear>;<isRadarEnabled>;<blueStatusLed>;<yellowStatusLed>;<desiredSpeedEDF>"
// Example: "1;10.5;150.6;1;60;1;1;1;100;1350"
void processSerialCommand(const char* command) {
  // Create local copies of the data for parsing
  String inputString = String(command);
  inputString.trim();  // Remove any leading/trailing whitespace
  
  // Find the semicolon separator
  int firstSeparator = inputString.indexOf(';');
  int secondSeparator = inputString.indexOf(';', firstSeparator + 1);
  int thirdSeparator = inputString.indexOf(';', secondSeparator + 1);
  int forthSeparator = inputString.indexOf(';', thirdSeparator + 1);
  int fifthSeparator = inputString.indexOf(';', forthSeparator + 1);
  int sixthSeparator = inputString.indexOf(';', fifthSeparator + 1);
  int seventhSeparator = inputString.indexOf(';', sixthSeparator + 1);
  int eightSeparator = inputString.indexOf(';', seventhSeparator + 1);
  int lastSemicolon = inputString.indexOf(';', eightSeparator + 1);

  String payload = inputString.substring(0, lastSemicolon);
  checksumStr = inputString.substring(lastSemicolon + 1);
  checksumCalc = 0;
  for (char c : payload) {
      checksumCalc += (uint8_t)c;
  }

  //if (checksumCalc == checksumStr.toInt()) 
  if (true)
  {
    if (firstSeparator != -1 && secondSeparator != -1 && thirdSeparator != -1
        && forthSeparator != -1 && fifthSeparator != -1 && sixthSeparator != -1 
        && seventhSeparator != -1 && eightSeparator != -1 && lastSemicolon != -1) {

      // If car is not stopped via Remote 
      if(!remoteControlOverride){
        String enableCarEngineString = inputString.substring(0, firstSeparator);
        enableCarEngine = enableCarEngineString.toInt();
      }

      String angleString = inputString.substring(firstSeparator + 1, secondSeparator);
      lastServoAngle = angleString.toFloat();

      String speedString = inputString.substring(secondSeparator + 1, thirdSeparator);
      desiredSpeed = speedString.toFloat();
      desiredSpeed = desiredSpeed > 0 ? (desiredSpeed < maxDesiredSpeed ? desiredSpeed :maxDesiredSpeed) : 0;

      
      String finishLineString = inputString.substring(thirdSeparator + 1, forthSeparator);
      if(1 == finishLineString.toInt())
      {
        isFinishLineDetected = true;
      }else{
        isFinishLineDetected = false;
      }

      String stringDistanceIssues = inputString.substring(forthSeparator + 1, fifthSeparator);
      distanceBeforeIssuesAppear = stringDistanceIssues.toInt();

      String stringIsRadarEnabled = inputString.substring(fifthSeparator + 1, sixthSeparator);
      if (1 == stringIsRadarEnabled.toInt())
      {
        isRadarEnabled = true;
      }
      else
      {
        isRadarEnabled = false;
      }

      String blueStatusLedString = inputString.substring(sixthSeparator + 1, seventhSeparator);
      if (1 == blueStatusLedString.toInt())
      {
        isBlueStatusLedOn = true;
      }
      else
      {
        isBlueStatusLedOn = false;
      }

      String yellowStatusLedString = inputString.substring(seventhSeparator + 1, eightSeparator);
      if (1 == yellowStatusLedString.toInt())
      {
        isYellowStatusLedOn = true;
      }
      else
      {
        isYellowStatusLedOn = false;
      }

      String speedStringEDF = inputString.substring(eightSeparator + 1, lastSemicolon);
      desiredSpeedEDF = speedStringEDF.toFloat();
      desiredSpeedEDF = desiredSpeedEDF > 0 ? (desiredSpeedEDF < maxDesiredSpeed ? desiredSpeedEDF :maxDesiredSpeed) : 0;
      
    }
  }
}

#if (1 == PID_SPEED_CALIBRATION)    
  void processSerialCommandPidCalibration(const char* command) {
  // Create local copies of the data for parsing
  String inputString = String(command);
  inputString.trim();  // Remove any leading/trailing whitespace

  
  // Find the semicolon separator
  int firstSeparator = inputString.indexOf(';');
  int secondSeparator = inputString.indexOf(';', firstSeparator + 1);
  int thirdSeparator = inputString.indexOf(';', secondSeparator + 1);
  int forthSeparator = inputString.indexOf(';', thirdSeparator + 1);

  if (firstSeparator != -1 && secondSeparator != -1 && thirdSeparator != -1
      && forthSeparator != -1) {

    String angleString = inputString.substring(0, firstSeparator);
    lastServoAngle = angleString.toFloat();

    String speedString = inputString.substring(firstSeparator + 1, secondSeparator);
    desiredSpeed = speedString.toFloat();
    desiredSpeed = desiredSpeed > 0 ? (desiredSpeed < maxDesiredSpeed ? desiredSpeed :maxDesiredSpeed) : 0;

    String kpString = inputString.substring(secondSeparator + 1, thirdSeparator);
    kp = kpString.toFloat();

    String kiString = inputString.substring(thirdSeparator + 1, forthSeparator);
    ki = kiString.toFloat();
    
    String kdString = inputString.substring(forthSeparator + 1);
    kd = kdString.toFloat();
  }
  else {
    // Used for Quick test (e.g. 105.5)
    desiredSpeed = inputString.toFloat();
    desiredSpeed = desiredSpeed > 0 ? (desiredSpeed < maxDesiredSpeed ? desiredSpeed :maxDesiredSpeed) : 0;
  }
}
#endif

void emergencyStopISR() {
    remoteControlOverride = true;
    enableCarEngine = false;
    desiredSpeed = 0;
    driverMotor.write(STANDSTILL_SPEED);
}

void emergencyStartISR() {
    enableCarEngine = true;
    remoteControlOverride = false;
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

/*void updateSpeedPID(float actualSpeed, float desiredSpeed) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDUpdate < PID_UPDATE_INTERVAL) {
        return; // Run PID at a fixed interval
    }
    lastPIDUpdate = currentMillis;

    error = desiredSpeed - actualSpeed;

    float P = kp * error;

    integral += error * (PID_UPDATE_INTERVAL / 1000.0);
    integral = constrain(integral, -50, 50);

    float I = ki * integral;

    float derivative = (error - previousError) / (PID_UPDATE_INTERVAL / 1000.0);
    float D = kd * derivative;

    previousError = error;

    float pidOutput = P + I + D;
    driverMotorSpeed += pidOutput;
    driverMotorSpeed = constrain(driverMotorSpeed, STANDSTILL_SPEED, MAX_MOTOR_SPEED);

    if (enableCarEngine) {
        driverMotor.write(driverMotorSpeed);
    } else {
        driverMotor.write(STANDSTILL_SPEED);
    }
}
*/
float speedToServoValue(float speed) {
  if (speed <= 0) return STANDSTILL_SPEED;
  
  float a = -0.0020;
  float b = 13.9775;
  float c = -1263.7335 - speed;
  
  float discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    // No real solution
    return -1;
  }

  // Use the '+' branch since servo value increases with speed
  float servoValue = (-b + sqrt(discriminant)) / (2 * a);
  return servoValue;
}


/*int getDistanceTfMiniPlus(){
  if (TFMINI_SERIAL.available() >= 9) { // Ensure we have a full packet
        uint8_t buf[9];
        
        TFMINI_SERIAL.readBytes(buf, 9);

        // Verify the packet header
        if (buf[0] == 0x59 && buf[1] == 0x59) {
            uint16_t distance = buf[2] | (buf[3] << 8);  // Distance in cm
            uint8_t checksum = buf[8];

            uint8_t sum = 0;
            for (int i = 0; i < 8; i++) {
                sum += buf[i];
            }

            if (sum == checksum) {
              return distance;
            }
        }
    }
    return -1;
}*/
int getDistanceTfMiniPlus(){
  int distance = -1;
  if (tfmini.readData()) {
      // Distance "default in CM"
      distance = tfmini.getDistance();
  }
  return distance;
}

void readLidar() {
    int distance = getDistanceTfMiniPlus();
    if (distance != -1) {
        distanceFromLidarInCm = distance;
    }
}


int calculateRadarServoAngle(int lastServoAngle){
  static bool currentDirection = true;
  if (lastServoAngle >= 30)
  {
    currentDirection = false;
  }
  else if ((lastServoAngle <= -30))
  {
    currentDirection = true;
  }

  if (currentDirection)
  {
    lastServoAngle++;
  }
  else
  {
    lastServoAngle--;
  }
  return lastServoAngle;
}
#include <Arduino.h>
#include <PWMServo.h>
#include "SteeringWheel.h"
#include <IntervalTimer.h>
#include "TFMiniPlus.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define CLAMP(val, low, high) MIN(MAX(val, low), high)

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
//#define DRIVER_MOTOR_PIN  9
#define DRIVER_MOTOR_PIN_LEFT  6
#define DRIVER_MOTOR_PIN_RIGHT  9

#define STANDSTILL_SPEED 90.0f
#define MAX_MOTOR_SPEED 180.0f

#define FINISH_LINE_SPEED 60                                                       // cm/s
#define MAX_DESIRED_SPEED 500//215                                                 // cm/s

#define WHEEL_BASE_M 0.145f           // meters

// ================== EDF_FAN ==================
#define EDF_FAN_PIN  5

// ================== RPM Sensor ==================
//#define RPM_SENSOR_PIN  14
//#define RPM_SENSOR_PIN_LEFT  14
//#define RPM_SENSOR_PIN_RIGHT  15

#define PULSES_PER_REV  16
#define RPM_MEDIAN_FILTER_SAMPLE_SIZE  10
#define DISTANCE_PER_REVOLUTION 7                                              // Revolution = 7 cm on the ground

// ================== Emergency Remote ==================
//#define REMOTE_START_PIN 15
//#define REMOTE_STOP_PIN 16

// ================== Status LED pin ==================
#define STATUS_LED_BLUE_PIN 20
#define STATUS_LED_YELLOW_PIN 21


SteeringWheel servoSteering(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
SteeringWheel radarServo(RADAR_SERVO_ANGLE_MAX_LEFT, RADAR_SERVO_ANGLE_MIDDLE, RADAR_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotorRight;  
PWMServo driverMotorLeft;
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
const unsigned long updateIntervalSteering = 10;                                // 20 ms(50hz) interval
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
static float RearWheelTurnRadius(float wheelBase, float turnAngle);
int AngleToDirectionDeg(float angle_deg);
void SetSoftwareDifferentialSpeed(float speed_ms, float turn_radius_m, int left_right_turn, float _distanceBwWheels_m);
	

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

    //pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
    //pinMode(REMOTE_STOP_PIN, INPUT_PULLDOWN);
    //pinMode(REMOTE_START_PIN, INPUT_PULLDOWN);

    //attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), pulseISR, FALLING);
    //attachInterrupt(digitalPinToInterrupt(REMOTE_STOP_PIN), emergencyStopISR, RISING);
    //attachInterrupt(digitalPinToInterrupt(REMOTE_START_PIN), emergencyStartISR, RISING);
    
    Serial.begin(BAUD_RATE);
    TFMINI_SERIAL.begin(TFMINI_BAUD_RATE);
    while(!Serial){
      delay(10);
    }
    while(!TFMINI_SERIAL){
      delay(10);
    }
    tfmini.begin(&TFMINI_SERIAL);

    driverMotorRight.attach(DRIVER_MOTOR_PIN_RIGHT, 1148, 1832); 
    driverMotorLeft.attach(DRIVER_MOTOR_PIN_LEFT, 1148, 1832); 
    edfFan.attach(EDF_FAN_PIN, 1148, 1832); 
    
    edfFan.write(STANDSTILL_SPEED);
    delay(7000);
}
void loop() {

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  parseSerialInput();

  setLedStatus(STATUS_LED_BLUE_PIN, isBlueStatusLedOn);
  setLedStatus(STATUS_LED_YELLOW_PIN, isYellowStatusLedOn);

  /* static unsigned long RpsLocalBuffer[RPM_MEDIAN_FILTER_SAMPLE_SIZE];

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
  */
 
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

  // float writeSpeedDriverMotor = speedToServoValue(desiredSpeed);
 
  // if (writeSpeedDriverMotor >= 0) {   
  //   driverMotorRight.write(writeSpeedDriverMotor);
  //   driverMotorLeft.write(writeSpeedDriverMotor);
  // }

  float g_steering_angle_rad = lastServoAngle * M_PI / 180.0f;
  float g_rear_axe_turn_radius_m = RearWheelTurnRadius(WHEEL_BASE_M, g_steering_angle_rad);

  int steering_direction = AngleToDirectionDeg(lastServoAngle);

  SetSoftwareDifferentialSpeed(desiredSpeed / 100.0f, g_rear_axe_turn_radius_m, steering_direction, WHEEL_BASE_M);


   float writeSpeedEDF = speedToServoValue(desiredSpeedEDF);
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
    driverMotorRight.write(STANDSTILL_SPEED);
    driverMotorLeft.write(STANDSTILL_SPEED);
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
// float speedToServoValue(float speed) {
//   if (speed <= 0) return STANDSTILL_SPEED;
  
//   float a = -0.0020;
//   float b = 13.9775;
//   float c = -1263.7335 - speed;
  
//   float discriminant = b * b - 4 * a * c;
//   if (discriminant < 0) {
//     // No real solution
//     return -1;
//   }

//   // Use the '+' branch since servo value increases with speed
//   float servoValue = (-b + sqrt(discriminant)) / (2 * a);
//   return servoValue;
// }

int floatCmp(float a, float b)
{
    const float abs_epsilon = 1e-6f;
    const float rel_epsilon = 1e-5f;

    // NaN handling
    if (isnan(a) || isnan(b))
        return 0;   // sau poți decide alt comportament

    // Handle infinities explicitly
    if (isinf(a) || isinf(b))
    {
        if (a == b) return 0;
        return (a < b) ? -1 : 1;
    }

    // Handle exact equality (+0 vs -0 included)
    if (a == b)
        return 0;

    float diff = a - b;
    float abs_diff = fabsf(diff);

    // Absolute tolerance
    if (abs_diff <= abs_epsilon)
        return 0;

    // Relative tolerance
    float max_ab = fmaxf(fabsf(a), fabsf(b));
    if (abs_diff <= max_ab * rel_epsilon)
        return 0;

    return (diff < 0.0f) ? -1 : 1;
}

static float RearWheelTurnRadius(float wheelBase, float turnAngle) {
	float angle;
	//float temp_sin = sinf(turnAngle);
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	float temp_sin = tanf(turnAngle);
	if (floatCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	//angle = (wheelBase / tanf(turnAngle));
	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}

// maxLeft:30 ; maxRight:-30
// -1: left, 0: forward, 1: right
	int AngleToDirectionDeg(float angle_deg) {
		int cmp_result = floatCmp(angle_deg, 0.0);
		if (cmp_result > 0) {
			return 1;
		}
		else if (cmp_result < 0.0) {
			return -1;
		}
		else {
			return 0;
		}
	}


// left_right_turn: negative if turning left, positive if turning right
void SetSoftwareDifferentialSpeed(float speed_ms, float turn_radius_m, int left_right_turn, float _distanceBwWheels_m){
		float left_wheel_turn_radius;
		float right_wheel_turn_radius;
		float left_wheel_turn_circonference;
		float right_wheel_turn_circonference;
		float car_turn_circumference;
		float left_wheel_speed_request_m;
		float right_wheel_speed_request_m;

		turn_radius_m = fabs(turn_radius_m);

		if (floatCmp(turn_radius_m, 0.0) == 0 || left_right_turn == 0)	// going straight
		{
			left_wheel_speed_request_m = speed_ms;
			right_wheel_speed_request_m = speed_ms;
		}
		else{
			if (left_right_turn < 0)	// left turn
			{
				left_wheel_turn_radius = turn_radius_m - (_distanceBwWheels_m / 2.0);
				right_wheel_turn_radius = turn_radius_m + (_distanceBwWheels_m / 2.0);
			}
			else if (left_right_turn > 0)	// right turn
			{
				left_wheel_turn_radius = turn_radius_m + (_distanceBwWheels_m / 2.0);
				right_wheel_turn_radius = turn_radius_m - (_distanceBwWheels_m / 2.0);
			}

			left_wheel_turn_circonference = (2.0 * left_wheel_turn_radius) * M_PI;
			right_wheel_turn_circonference = (2.0 * right_wheel_turn_radius) * M_PI;
			car_turn_circumference = (2.0 * turn_radius_m) * M_PI;
			
			left_wheel_speed_request_m =(left_wheel_turn_circonference / car_turn_circumference) * speed_ms;
			right_wheel_speed_request_m =(right_wheel_turn_circonference / car_turn_circumference) * speed_ms;
		}
		

  float left_motor_speed_raw = speedToServoValue(left_wheel_speed_request_m * 100.0f);
  float right_motor_speed_raw = speedToServoValue(right_wheel_speed_request_m * 100.0f);

 
    driverMotorRight.write(right_motor_speed_raw);
    driverMotorLeft.write(left_motor_speed_raw);
	}


float MeterPerSecondToESCValue(float speed_mps){
  float ESC_standstill_value = STANDSTILL_SPEED; // ESC value for standstill
  float ESC_max_value = MAX_MOTOR_SPEED; // ESC value for maximum speed

  float wheel_diameter_m = 0.063f; // in meters
  float motor_kv = 980.0f;
  float battery_viltage_v = 8.4f;
  float little_gear_total_theeths = 17.0f;
  float big_gear_total_theeths = 84.0f;

  float wheel_circumference_m = 3.14159f * wheel_diameter_m;
  float wheel_rpm = (speed_mps / wheel_circumference_m) * 60.0f;
  float motor_rpm = wheel_rpm * (big_gear_total_theeths / little_gear_total_theeths);
  float motor_speed_fraction = motor_rpm / (motor_kv * battery_viltage_v);

  float esc_value = ESC_standstill_value + (ESC_max_value - ESC_standstill_value) * motor_speed_fraction;
  return (float)esc_value;
}


// speed in cm/s
float speedToServoValue(float speed){
  if (speed <= 0.001f) return STANDSTILL_SPEED;
  
  float escValue = MeterPerSecondToESCValue(speed / 100.0f); // Convert cm/s to m/s
  escValue = MAX(escValue, STANDSTILL_SPEED);
  return escValue;
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
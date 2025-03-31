#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CAMERA_CALIBRATION 0
#define ENABLE_CAMERA_THRESHOLD_CHECK 0
#define ENABLE_CAMERA_STREAMING 1
#define ENABLE_TCP_OUTPUT 0
#define ENABLE_TEENSY_SERIAL 1
#define ENABLE_FINISH_LINE_DETECTION 1
#define DEFAULT_START_RACE 0  // When set to 1 car starts 

//#define SERIAL_PORT "/dev/ttymxc2"      
#define SERIAL_PORT "/dev/ttyACM0"      

/*

for indoor track
84 99
-49 229
242 99
353 229
151 239

240x180
35 73
-57 171
163 73
250 171
96 179

200x150
44 61
-22 142
164 61
235 142
106 149

for B020 curved track
74 84
-71 229
231 84
373 229
150 239

64 99
-54 229
238 99
369 229
158 239
*/

#define GPIO_CHIP "/dev/gpiochip0"
#define SW_LINE 12
#define SHM_NAME "/config_shared_memory"
#define SHM_SIZE sizeof(SharedConfig)

// Range field is added for the LCD limits !!DO NOT DELETE
#pragma pack(push, 1)  // Disable struct padding
struct SharedConfig {
    int startRace;                                          //Range: 0 - 1
    int enableCarEngine;                                    //Range: 0 - 1
    int enableCarSteering;                                  //Range: 0 - 1
    int enableCameraThresholdCheck;                         //Range: 0 - 1
    int enableFinishLineDetection;                          //Range: 0 - 1
    int thresholdValue;                                     //Range: 0 - 255
    int distanceErrorFromChassis;                           //Range: 0 - 240
    int lineMinPixelCount;                                  //Range: 0 - 255
    int distanceSensorError;                                //Range: 0 - 30
    int stoppingDistanceBoxFrontEnd;                        //Range: 1 - 9
    char _padding[8];
    double calibrateTopLinePerc;                            //Range: 0 - 100
    double calibrateBottomLinePerc;                         //Range: 0 - 100
    double trackLaneWidthOffset;                            //Range: -100 - 200
    double topImageCutPercentage;                           //Range: 0 - 1
    double bottomImageCutPercentage;                           //Range: 0 - 1
    double topCutOffPercentageCustomConnected;              //Range: 0 - 1
    double line90DegreeAngleRange;                          //Range: 0 - 90
    double finishLineAngleRange;                            //Range: 90 - 180
    double afterFinishLineSpeed;                            //Range: 0 - 350
    double servoTurnAdjustmentCoefficient;                  //Range: 0 - 5
    double corneringSpeedCoefficient;                       //Range: 0 - 2
    double minSpeed;                                        //Range: 0 - 350
    double maxSpeed;                                        //Range: 0 - 350
    double curvatureFactor;                                 //Range: 0 - 200
    double k_min;                                           //Range: 0 - 25
    double k_max;                                           //Range: 0 - 25
    double R_minInCm;                                       //Range: 0 - 2000
    double R_maxInCm;                                       //Range: 0 - 2000
    double minLookAheadInCm;                                //Range: 0 - 100
    double maxLookAheadInCm;                                //Range: 0 - 100
    double waitBeforeStartSeconds;                          //Range: 0 - 10
    double straightWheelTimerSeconds;                       //Range: 0 - 5
};
#pragma pack(pop)  // Restore default padding


// ---------------------------------------- USED IN SHARED MEMORY ----------------------------------------------------------
// Integer values
#define DEFAULT_ENABLE_CAR_ENGINE 0
#define DEFAULT_ENABLE_CAR_STEERING 1
#define DEFAULT_THRESHOLD_VALUE 51
#define DEFAULT_DISTANCE_ERROR_FROM_CHASSIS 0
#define DEFAULT_LINE_MIN_PIXEL_COUNT 70 
#define DEFAULT_DISTANCE_FROM_SENSOR_ERROR 27
#define DEFAULT_STOPPING_DISTANCE_BOX_FRONT_END 0

// Double values
#define DEFAULT_CALIBRATE_TOP_LINE 41.6 //100
#define DEFAULT_CALIBRATE_BOTTOM_LINE 95.8 //230
#define DEFAULT_TRACK_LANE_WIDTH_OFFSET 0.0
#define DEFAULT_TOP_IMAGE_CUT_PERCENTAGE 0.0
#define DEFAULT_BOTTOM_IMAGE_CUT_PERCENTAGE 0.35
#define DEFAULT_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED 0.45 // Cuts pixels from first 45% of image 
#define DEFAULT_BOTTOM_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED 1 //0.65
#define DEFAULT_LINE_90_DEGREE_ANGLE_RANGE 20.0                          // abs(degree-90) < range
#define DEFAULT_FINISH_LINE_ANGLE_RANGE 15.0
#define DEFAULT_AFTER_FINISH_LINE_SPEED 50.0
#define DEFAULT_SERVO_TURN_ADJUSTMENT_COEFFICIENT 1.0
#define DEFAULT_CORNERING_SPEED_COEFFICIENT 0.9
#define DEFAULT_MIN_SPEED 150.0
#define DEFAULT_MAX_SPEED 250.0
#define DEFAULT_CURVATURE_FACTOR 13.0
#define DEFAULT_K_MIN 14.8
#define DEFAULT_K_MAX 18.5
#define DEFAULT_R_MIN_IN_CM 20.0
#define DEFAULT_R_MAX_IN_CM 3000.0
#define DEFAULT_MIN_LOOKAHEAD_IN_CM 40.0
#define DEFAULT_MAX_LOOKAHEAD_IN_CM 55.0
#define DEFAULT_WAIT_BEFORE_START_SECONDS 5.0
#define DEFAULT_STRAIGHT_WHEEL_TIMER_SECONDS 2.0


/*
// Global values
int enableCarEngine = 0;   // 0 or 1
int enableCameraThresholdCheck = ENABLE_CAMERA_THRESHOLD_CHECK;   // 0 or 1

// Used to do a simple threshold
int thresholdValue = 60;                 // 80 - B020;                             
constexpr int maxThresholdValue = 255;

// Used to get the points that intersect the lines at rows set below
int calibrateTopLine = 100;
int calibrateBottomLine = 230;

// Used to resize frame
double topImageCutPercentage = 0; //0.35; 

// Used to calculate where chassis should be in image
int distanceErrorFromChassis = 0;                    // Measured in Pixels

// Used in customConnectedComponentsWithThreshold()
int lineMinPixelCount = 45;                           // 45 Defines how many pixel can make a line (removes noise)  finish lines sizes: 57 64       
double topCutOffPercentageCustomConnected = 0.35;      // Top 40% cutoff to mitigate Far View error                (Range: 0.0 - 1.0)

// Used in removeHorizontalIf90Turn()
double min90DegreeAngleRange = 70;                    // Values between min and max ar cut off to mitigate error
double max90DegreeAngleRange = 110;                   // Values between min and max ar cut off to mitigate error

// Used in mapAngleToServo()
double servoTurnAdjustmentCoefficient = 1;          // Used to adjust car's turning  (1.0 = 100%)

// Used in calculateServoValue() PurePursuitAlgo
double minSpeed = 200.0;                                // Vehicle speed min 0 cm/s
double maxSpeed = 500.0;                               // Vehicle speed max 40 cm/s
double curvatureFactor = 10.0;                        // Tunable factor for sensitivity
double k_min = 0.1;                                   // Lookahead distance tuning constant
double k_max = 0.4;                                   // Lookahead distance tuning constant
double R_minInCm = 20;                                // Road Curvature Radius min
double R_maxInCm = 1000;                              // Road Curvature Radius max
double minLookAheadInCm = 15.0;                       // Minimum lookahead distance in cm
double maxLookAheadInCm = 40.0;                      // Maximum lookahead distance in cm

//12s run
// Used in calculateServoValue() PurePursuitAlgo
// double minSpeed = 200.0;                                // Vehicle speed min 0 cm/s
// double maxSpeed = 500.0;                               // Vehicle speed max 40 cm/s
// double curvatureFactor = 10.0;                        // Tunable factor for sensitivity
// double k_min = 0.1;                                   // Lookahead distance tuning constant
// double k_max = 0.4;                                   // Lookahead distance tuning constant
// double R_minInCm = 20;                                // Road Curvature Radius min
// double R_maxInCm = 1000;                              // Road Curvature Radius max
// double minLookAheadInCm = 15.0;                       // Minimum lookahead distance in cm
// double maxLookAheadInCm = 40.0;                      // Maximum lookahead distance in cm

*/
// ---------------------------------------- CONSTANS ----------------------------------------------------------
// Used to setup Camera
constexpr int captureFrameWidth = 320;
constexpr int captureFrameHeight = 240;
constexpr int captureTotalPixels = captureFrameWidth * captureFrameHeight;

constexpr int resizeFrameWidth = 200;//240;//320;
constexpr int resizeFrameHeight = 150;//180;//240;
constexpr int resizeTotalPixels = resizeFrameWidth * resizeFrameHeight;
constexpr double ScalingFactor = static_cast<double>(resizeTotalPixels) / captureTotalPixels;
constexpr int  _minLinePixelCount = static_cast<int>(ScalingFactor * DEFAULT_LINE_MIN_PIXEL_COUNT);

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = static_cast<int>(35 * ScalingFactor);  
constexpr double fitPolyEpsilon = static_cast<double>(14.0 * ScalingFactor); // Epsilon value for curve approximation

constexpr int captureFps = 100;
cv::Point2f undefinedPoint = cv::Point2f(1000,0);

constexpr int distanceBeforeIssuesAppear = 90;

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;                   // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;                  // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

constexpr int maxThresholdValue = 255;
constexpr double APPROACHING_INTERSECTION_minLineLength = 75;  

constexpr int distanceMedianFilterSampleSize = 5;

// Used in findMiddle()
constexpr int curveSamplePoints = 15;                           // 15 Number of points to sample the curve(High number equals more complexity)

// Used in are2PointsHorizontal()
constexpr double horizontalSlopeThreshold = 1;                  // Absolute value to handle horizontal line cutoffs

// used to calculate pixelSizeInCm
constexpr double trackWidthInCm = 53.0;                         // Track Width is 53 Cm

// Used in  calculateServoValue()
constexpr double wheelBaseInCm = 17.0;                          // Distance between front and rear axle is 17cm

// Used in mapAngleToServo()
constexpr double maxSteeringAngleDegrees = 50.0;                // Maximum steering angle in degrees either to the left or right
constexpr double maxServoAngle = 30.0;                          // Used to limit servo rotation
constexpr double maxLeftServoAngle = -30.0;                     // Used to limit servo rotation
constexpr double maxRightServoAngle = 30.0;                     // Used to limit servo rotation

// Used in processFrames()
constexpr double overlayFrameWeight = 1.0;                      // Used for visualization of two frames on top of eachother

#endif
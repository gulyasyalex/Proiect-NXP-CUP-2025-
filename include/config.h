#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CAMERA_CALIBRATION 0
#define ENABLE_CAMERA_THRESHOLD_CHECK 0
#define ENABLE_CAMERA_STREAMING 1
#define ENABLE_TCP_FRAMES 1
#define ENABLE_TCP_SITE_DEBUG 1
#define ENABLE_TEENSY_SERIAL 1
#define ENABLE_FINISH_LINE_DETECTION 0 //THIS is set to 1 by timer after START_RACE
#define DEFAULT_START_RACE 0  // When set to 1 car starts 

//#define SERIAL_PORT "/dev/ttymxc2"      
//#define SERIAL_PORT "/dev/ttyACM0"      
#define SERIAL_PORT "/dev/serial/by-id/usb-Teensyduino_USB_Serial_15146110-if00"      

/*

//NEAR View Camera 200x150 setup
29 61
-43 142
157 61
221 142
88 149

43 61
-38 142
147 61
209 142
84 149

//FAR View Camera 200x150 setup
45 53
-31 135
139 53
208 135
87 149

57 53
-26 135
145 53
207 135
89 149



// NXP BUCHAREST
39 53
-44 135
145 53
214 135
83 149

*/

#define GPIO_CHIP "/dev/gpiochip0"
#define SW_LINE 12
#define SHM_NAME "/config_shared_memory"
#define SHM_SIZE sizeof(SharedConfig)

enum State {
    FOLLOWING_LINE = 0,
    IN_INTERSECTION,
    EXITING_INTERSECTION
};

enum InterpolatedPointsSetup {
    NEAR_VIEW_SETUP = 0,
    FAR_VIEW_SETUP
};

// Range field is added for the LCD limits !!DO NOT DELETE
#pragma pack(push, 1)  // Disable struct padding
struct SharedConfig {
    int startRace;                                          //Range: 0 - 1
    int enableCarEngine;                                    //Range: 0 - 1
    int enableCarSteering;                                  //Range: 0 - 1
    int enableCameraThresholdCheck;                         //Range: 0 - 1
    int enableFinishLineDetection;                          //Range: 0 - 1
    int currentState;                                       //Range: 0 - 3
    int thresholdValue;                                     //Range: 0 - 255
    int distanceErrorFromChassis;                           //Range: -240 - 240
    int lineMinPixelCount;                                  //Range: 0 - 255
    int distanceSensorError;                                //Range: 0 - 30
    int stoppingDistanceBoxFrontEnd;                        //Range: 1 - 30
    int interpolatedPointsSetup;                            //Range: 0 - 1
    char _padding[8];
    double calibrateTopLinePerc;                            //Range: 0 - 100
    double calibrateBottomLinePerc;                         //Range: 0 - 100
    double trackLaneWidthOffset;                            //Range: -100 - 200
    double topImageCutPercentage;                           //Range: 0 - 1
    double topCutOffPercentageCustomConnected;              //Range: 0 - 1
    double lineStartPointY;                                 //Range: 0 - 1
    double line90DegreeAngleRange;                          //Range: 0 - 90
    double finishLineAngleRange;                            //Range: 90 - 180
    double servoTurnAdjustmentCoefficient;                  //Range: 0 - 5
    double corneringSpeedCoefficient;                       //Range: 0 - 2
    double minSpeed;                                        //Range: 0 - 350
    double maxSpeed;                                        //Range: 0 - 350
    double minSpeedAfterFinish;                             //Range: 0 - 350
    double maxSpeedAfterFinish;                             //Range: 0 - 350
    double currentEdfFanSpeed;                              //Range: 0 - 350
    double curvatureFactor;                                 //Range: 0 - 200
    double rdp_epsilon;                                     //Range: 0 - 25
    double k_max;                                           //Range: 0 - 25
    double minAngleLookAheadReference;                      //Range: 0 - 180
    double maxAngleLookAheadReference;                                       //Range: 0 - 180
    double minLookAheadInCm;                                //Range: 0 - 100
    double maxLookAheadInCm;                                //Range: 0 - 100
    double waitBeforeStartSeconds;                          //Range: 0 - 10
    double waitBeforeEdfStartSeconds;                       //Range: 0 - 5
    double waitBeforeFinishDetectionSeconds;                //Range: 0 - 20
};
#pragma pack(pop)  // Restore default padding


// ---------------------------------------- USED IN SHARED MEMORY ----------------------------------------------------------
// Integer values
#define DEFAULT_ENABLE_CAR_ENGINE 0
#define DEFAULT_ENABLE_CAR_STEERING 0
#define DEFAULT_THRESHOLD_VALUE 170 //75 //150
#define DEFAULT_DISTANCE_ERROR_FROM_CHASSIS 0
#define DEFAULT_LINE_MIN_PIXEL_COUNT 70 
#define DEFAULT_DISTANCE_FROM_SENSOR_ERROR 10
#define DEFAULT_STOPPING_DISTANCE_BOX_FRONT_END 3
#define DEFAULT_INTERPOLATED_POINTS_SETUP 0         // 0 - Near View Setup 1 - Far View Setup (BirdEyeView)

// Double values
#define DEFAULT_CALIBRATE_TOP_LINE 36.6 //41.6(percentage) //100
#define DEFAULT_CALIBRATE_BOTTOM_LINE 90.8 //95.8(percentage) //230
#define DEFAULT_TRACK_LANE_WIDTH_OFFSET -8.4 //SET TO 0 AT FINALS
#define DEFAULT_TOP_IMAGE_CUT_PERCENTAGE 0.0
#define DEFAULT_BOTTOM_IMAGE_CUT_PERCENTAGE 0.35
#define DEFAULT_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED 0.4 // Cuts pixels from first 45% of image 
#define DEFAULT_LINE_START_POINT_Y 0.50 //0.60 // Used for intersection // birdsEyeViewHeight * lineStartPointY = Y threshold
#define DEFAULT_BOTTOM_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED 1 //0.65
#define DEFAULT_LINE_90_DEGREE_ANGLE_RANGE 22.0                          // abs(degree-90) < range
#define DEFAULT_FINISH_LINE_ANGLE_RANGE 15.0
#define DEFAULT_SERVO_TURN_ADJUSTMENT_COEFFICIENT 1.4 //1.0
#define DEFAULT_CORNERING_SPEED_COEFFICIENT 1.6 //0.6
#define DEFAULT_MIN_SPEED 80.0
#define DEFAULT_MAX_SPEED 320.0
#define DEFAULT_MIN_SPEED_AFTER_FINISH 35.0
#define DEFAULT_MAX_SPEED_AFTER_FINISH 40.0
#define DEFAULT_EDF_FAN_CURRENT_SPEED 350.0
#define DEFAULT_CURVATURE_FACTOR 13.0
#define DEFAULT_RDP_EPSILON 18
#define DEFAULT_K_MAX 25.5 //55 // 25.5  //18.25
#define DEFAULT_MIN_ANGLE_LOOKAHEAD_REFERENCE 0.0
#define DEFAULT_MAX_ANGLE_LOOKAHEAD_REFERENCE 50.0
#define DEFAULT_MIN_LOOKAHEAD_IN_CM 40.0
#define DEFAULT_MAX_LOOKAHEAD_IN_CM 65.0
#define DEFAULT_WAIT_BEFORE_START_SECONDS 4.0
#define DEFAULT_WAIT_BEFORE_EDF_START_SECONDS 0.5
#define DEFAULT_WAIT_BEFORE_FINISH_DETECTION_SECONDS 7

// OTHER DEFAULTS
#define DEFAULT_AFTER_FINISH_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED 0.4 // Cuts pixels from first 45% of image 
#define DEFAULT_EDF_FAN_AFTER_FINISH_SPEED 350.0
#define DEFAULT_TORQUE_SPEED 60;

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

//constexpr double  lineStartPointY = 0.60;    // Used for intersection // birdsEyeViewHeight * lineStartPointY = Y threshold

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = static_cast<int>(35 * ScalingFactor);  
constexpr double fitPolyEpsilon = static_cast<double>(9.0 * ScalingFactor); // Epsilon value for curve approximation

constexpr int captureFps = 100; // 120 for BUCHAREST;
const cv::Point2f undefinedPoint(0.0f, 0.0f);

constexpr int distanceBeforeIssuesAppear = 60;

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;                   // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;                  // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

constexpr int maxThresholdValue = 255;
constexpr double INTERSECTION_minLineLength = 45;  
constexpr double IN_INTERSECTION_minLineLength = 60;

constexpr int distanceMedianFilterSampleSize = 5;

// Used in findMiddle()
constexpr int curveSamplePoints = 12;                           // 15 Number of points to sample the curve(High number equals more complexity)

// Used in computeCurvatureRadiusInFrontOfCar()
constexpr int falsePositiveCurvatureRadius = 90;               // When Calculating lookahead false Positives appear in radius Calculus

// Used in are2PointsHorizontal()
constexpr double horizontalSlopeThreshold = 1;                  // Absolute value to handle horizontal line cutoffs

// used to calculate pixelSizeInCm
constexpr double trackWidthInCm = 53.0;                         // Track Width is 53 Cm

// Used in  CalculateCarSpeed()
constexpr double wheelBaseInCm = 17.0;                          // Distance between front and rear axle is 17cm
constexpr double downward_accelerationCm = 981.0;               

// Used in mapAngleToServo()
constexpr double maxSteeringAngleDegrees = 50.0;                // Maximum steering angle in degrees either to the left or right
constexpr double maxServoAngle = 30.0;                          // Used to limit servo rotation
constexpr double maxLeftServoAngle = -30.0;                     // Used to limit servo rotation
constexpr double maxRightServoAngle = 30.0;                     // Used to limit servo rotation

// Used in processFrames()
constexpr double overlayFrameWeight = 1.0;                      // Used for visualization of two frames on top of eachother

#endif
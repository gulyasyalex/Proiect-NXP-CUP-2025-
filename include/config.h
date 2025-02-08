#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CAMERA_CALIBRATION 0
#define ENABLE_CAMERA_THRESHOLD_CHECK 0
#define ENABLE_CAMERA_STREAMING 1
#define ENABLE_TCP_OUTPUT 0
#define ENABLE_TEENSY_SERIAL 1

//#define SERIAL_PORT "/dev/ttymxc2"      
#define SERIAL_PORT "/dev/ttyACM0"      

/*

for indoor track
72 99
-43 229
232 99
354 229
156 239


for B020 curved track
74 84
-71 229
231 84
373 229
150 239
*/


// Global values
int enableCarEngine = 0;   // 0 or 1
int enableCameraThresholdCheck = ENABLE_CAMERA_THRESHOLD_CHECK;   // 0 or 1

// ---------------------------------------- VARIABLES ----------------------------------------------------------

// Used to do a simple threshold
#if 1 == ENABLE_CAMERA_THRESHOLD_CHECK
    constexpr int thresholdValue = 60;                 // 80 - B020;                             
    constexpr int maxThresholdValue = 255;
#else
    int thresholdValue = 60;                 // 80 - B020;                             
    int maxThresholdValue = 255;
#endif

#if 1 == ENABLE_CAMERA_CALIBRATION
    // Used to get the points that intersect the lines at rows set below
    int calibrateTopLine = 100;
    int calibrateBottomLine = 230;
#else
    // Used to get the points that intersect the lines at rows set below
    constexpr int calibrateTopLine = 100;
    constexpr int calibrateBottomLine = 230;
#endif
// Used to resize frame
double topImageCutPercentage = 0; //0.35; 

// Used to calculate where chassis should be in image
int distanceErrorFromChassis = 0;                    // Measured in Pixels

// Used in customConnectedComponentsWithThreshold()
int lineMinPixelCount = 45;                           // 45 Defines how many pixel can make a line (removes noise)  finish lines sizes: 57 64       
double topCutOffPercentageCustomConnected = 0.35;      // Top 40% cutoff to mitigate Far View error                (Range: 0.0 - 1.0)
double lineBottomStartRangeCustomConnected = 0.6;     // It cuts the top 40% so Bottom 60% Range for line group start point searching    (Range: 0.0 - 1.0)

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
// double maxLookAheadInCm = 40.0;                      // Maximum lookahead distance in cm*/


// ---------------------------------------- CONSTANS ----------------------------------------------------------
// Used to setup Camera
constexpr int captureFrameWidth = 320;
constexpr int captureFrameHeight = 240;
constexpr int resizeFrameWidth = 320;
constexpr int resizeFrameHeight = 240;
constexpr int captureFps = 100;
cv::Point2f undefinedPoint = cv::Point2f(1000,0);

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;                   // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;                  // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = 35;  
constexpr double fitPolyEpsilon = 9.0;                          // Epsilon value for curve approximation

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
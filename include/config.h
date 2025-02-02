#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CAMERA_CALIBRATION 0
#define ENABLE_CAMERA_THRESHOLD_CHECK 0
#define ENABLE_CAMERA_STREAMING 1
#define ENABLE_TCP_OUTPUT 1
#define ENABLE_TEENSY_SERIAL 1

//#define SERIAL_PORT "/dev/ttymxc2"      
#define SERIAL_PORT "/dev/ttyACM0"      

/*

for indoor track
73 19
-36 144
245 19
360 144
162 185

for B020 curved track
74 84
-71 229
231 84
373 229
150 239
*/


// Global values
constexpr int enableCameraThresholdCheck = ENABLE_CAMERA_THRESHOLD_CHECK;   // 0 or 1
constexpr int captureFrameWidth = 320;
constexpr int captureFrameHeight = 240;
constexpr int resizeFrameWidth = 320;
constexpr int resizeFrameHeight = 240;
constexpr int captureFps = 100;

// used to calculate pixelSizeInCm
double trackWidthInCm = 53.0;                     // 53cm

// Used to resize frame
double topCutOffPercentage = 0; //0.35; 
cv::Point2f undefinedPoint = cv::Point2f(1000,0);

// Used to calculate where chassis should be in image
constexpr int distanceErrorFromChassis = 0;                    // Measured in Pixels

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;                   // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;                  // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

// Used to get the points that intersect the lines at rows set below
constexpr int calibrateTopLine = 85;
constexpr int calibrateBottomLine = 230;

// Used to do a simple threshold
constexpr int thresholdValue = 80;                 // 75;                             
constexpr int maxThresholdValue = 255;

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = 35;  
constexpr double fitPolyEpsilon = 9.0;                          // Epsilon value for curve approximation

// Used in findMiddle()
int curveSamplePoints = 15;                           // 15 Number of points to sample the curve(High number equals more complexity)

// Used in are2PointsHorizontal()
constexpr double horizontalSlopeThreshold = 1;                  // Absolute value to handle horizontal line cutoffs

// Used in customConnectedComponentsWithThreshold()
int lineMinPixelCount = 45;                           // 45 Defines how many pixel can make a line (removes noise)  finish lines sizes: 57 64       
constexpr double topCutOffPercentageCustomConnected = 0.3;      // Top 40% cutoff to mitigate Far View error                (Range: 0.0 - 1.0)
constexpr double lineBottomStartRangeCustomConnected = 0.6;     // It cuts the top 40% so Bottom 60% Range for line group start point searching    (Range: 0.0 - 1.0)

// Used in removeHorizontalIf90Turn()
constexpr double min90DegreeAngleRange = 70;                    // Values between min and max ar cut off to mitigate error
constexpr double max90DegreeAngleRange = 110;                   // Values between min and max ar cut off to mitigate error

// Used in processFrames()
constexpr double overlayFrameWeight = 1.0;                      // Used for visualization of two frames on top of eachother



// Used in calculateServoValue() PurePursuitAlgo
double wheelBaseInCm = 17.0;                          //  17 Distance between fron and rear axle is 17cm)
double minSpeed = 0.0;                                // Vehicle speed min 0 cm/s
double maxSpeed = 700.0;                               // Vehicle speed max 40 cm/s
double curvatureFactor = 10.0;                        // Tunable factor for sensitivity
double k_min = 0.2;                                   // Lookahead distance tuning constant
double k_max = 0.5;                                   // Lookahead distance tuning constant
double R_minInCm = 20;                                // Road Curvature Radius min
double R_maxInCm = 1000;                              // Road Curvature Radius max
double minLookAheadInCm = 30.0;                       // Minimum lookahead distance in cm
double maxLookAheadInCm = 100.0;                      // Maximum lookahead distance in cm

// Used in mapAngleToServo()
constexpr double maxSteeringAngleDegrees = 50.0;                // Maximum steering angle in degrees either to the left or right
double servoTurnAdjustmentCoefficient = 1.5;          // Used to adjust car's turning  (1.0 = 100%)
constexpr double maxServoAngle = 35.0;                          // Used to limit servo rotation
constexpr double maxLeftServoAngle = -30.0;                     // Used to limit servo rotation
constexpr double maxRightServoAngle = 30.0;                     // Used to limit servo rotation

#endif
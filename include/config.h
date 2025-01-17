#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CAMERA_CALIBRATION 0
#define ENABLE_CAMERA_STREAMING 1
#define ENABLE_CAMERA_THRESHOLD_CHECK 0
#define ENABLE_TCP_OUTPUT 0

//#define SERIAL_PORT "/dev/ttymxc2"      
#define SERIAL_PORT "/dev/ttyACM0"      

/*
with CutOff 0.35
76 20
-45 120
245 20
345 120
146 151

57 110
-89 235
238 110
365 235
139 235
*/

// Global values
constexpr int captureFrameWidth = 320;
constexpr int captureFrameHeight = 240;
constexpr int resizeFrameWidth = 320;
constexpr int resizeFrameHeight = 240;
constexpr int captureFps = 100;

// Used to resize frame
constexpr double topCutOffPercentage = 0.35; 
cv::Point2f undefinedPoint = cv::Point2f(1000,0);

// Used to calculate where chassis should be in image
constexpr int distanceErrorFromChassis = 30;                    // Measured in Pixels

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;                   // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;                  // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

// Used to get the points that intersect the lines at rows set below
constexpr int calibrateTopLine = 20;
constexpr int calibrateBottomLine = 145;

// Used to do a simple threshold
constexpr int thresholdValue = 50;                             // 70 75
constexpr int maxThresholdValue = 255;

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = 35;  
constexpr double fitPolyEpsilon = 9.0;                          // Epsilon value for curve approximation

// Used in findMiddle()
constexpr int curveSamplePoints = 15;                           // Number of points to sample the curve(High number equals more complexity)

// Used in are2PointsHorizontal()
constexpr double horizontalSlopeThreshold = 1;                  // Absolute value to handle horizontal line cutoffs

// Used in customConnectedComponentsWithThreshold()
constexpr int lineMinPixelCount = 45;                           // Defines how many pixel can make a line (removes noise)  finish lines sizes: 57 64       
constexpr double topCutOffPercentageCustomConnected = 0.0;      // Top 40% cutoff to mitigate Far View error                (Range: 0.0 - 1.0)
constexpr double lineBottomStartRangeCustomConnected = 0.3;     // It cuts the top 40% so Bottom 60% Range for line group start point searching    (Range: 0.0 - 1.0)

// Used in removeHorizontalIf90Turn()
constexpr double min90DegreeAngleRange = 70;                    // Values between min and max ar cut off to mitigate error
constexpr double max90DegreeAngleRange = 110;                   // Values between min and max ar cut off to mitigate error

// Used in processFrames()
constexpr double overlayFrameWeight = 1.0;                      // Used for visualization of two frames on top of eachother



// Used in calculateServoValue()
constexpr double speed = 40.0;                                  // Vehicle speed (example: 30 cm/s)
constexpr double k = 3;                                       // Lookahead distance tuning constant
constexpr double minLookAhead = 30.0;                           // Minimum lookahead distance in cm
constexpr double maxLookAhead = 150.0;                          // Maximum lookahead distance in cm
constexpr double wheelBase = 17.0;                              // Wheelbase in cm

// Used in mapAngleToServo()
constexpr double maxSteeringAngleDegrees = 50.0;                // Maximum steering angle in degrees either to the left or right
constexpr double servoTurnAdjustmentCoefficient = 1.5;          // Used to adjust car's turning  (1.0 = 100%)
constexpr double maxServoAngle = 35.0;                          // Used to limit servo rotation
constexpr double maxLeftServoAngle = -30.0;                     // Used to limit servo rotation
constexpr double maxRightServoAngle = 30.0;                     // Used to limit servo rotation

#endif
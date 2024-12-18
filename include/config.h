#ifndef CONFIG_H
#define CONFIG_H


#include "SerialPortFunctions/SerialPort.hpp"

extern debix::SerialPort& serial; 

#define ENABLE_CALIBRATE_CAMERA 0
#define ENABLE_STREAMING 0

#define SERIAL_PORT "/dev/ttymxc2"      

// Global values
constexpr int captureFrameWidth = 1920;
constexpr int captureFrameHeight = 1080;
constexpr int captureFps = 60;

// Used to resize frame
constexpr int frameWidth = 320;
constexpr int frameHeight = 180;
cv::Point2f undefinedPoint = cv::Point2f(1000,0);

// Used to change perspective to TOP VIEW
constexpr double widthDstPoints = 0.45 * 320;           // It should be a box so it has same width and height
constexpr double heightDstPoints = 0.45 * 320;          // It should be a box so it has same width and height
constexpr double birdsEyeViewWidth = 370;
constexpr double birdsEyeViewHeight = 400;

// Used to get the points that intersect the lines at rows set below
constexpr int calibrateTopLine = 45;
constexpr int calibrateBottomLine = 170;

// Used to do a simple threshold
constexpr int thresholdValue = 90;
constexpr int maxThresholdValue = 255;

// Used in fitPolinomial()
constexpr int fitPolyWindowSize = 35;  
constexpr double fitPolyEpsilon = 9.0;                  // Epsilon value for curve approximation

// Used in findMiddle()
constexpr int curveSamplePoints = 12;                   // Number of points to sample the curve(High number equals more complexity)

// Used in are2PointsHorizontal()
constexpr double horizontalSlopeThreshold = 1;          // Absolute value to handle horizontal line cutoffs

// Used in customConnectedComponentsWithThreshold()
constexpr int lineMinPixelCount = 10;                   // Defines how many pixel can make a line (removes noise)         
constexpr double topCutOffPercentage = 0.4;             // Top 40% cutoff to mitigate Far View error

// Used in removeHorizontalIf90Turn()
constexpr double min90DegreeAngleRange = 70;            // Values between min and max ar cut off to mitigate error
constexpr double max90DegreeAngleRange = 110;           // Values between min and max ar cut off to mitigate error

// Used in processFrames()
constexpr double overlayFrameWeight = 1.0;              // Used for visualization of two frames on top of eachother

// Used in mapAngleToServo()
constexpr double maxLeftServoAngle = -30.0;                  // Used to limit servo rotation
constexpr double maxRightServoAngle = 30.0;                  // Used to limit servo rotation

#endif
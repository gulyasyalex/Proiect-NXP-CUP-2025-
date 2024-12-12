#ifndef CONFIG_H
#define CONFIG_H


#define ENABLE_CALIBRATE_CAMERA 0
#define ENABLE_STREAMING 0

// Global values
    static int cutHeight = 0;
    static const int frameWidth = 320; //160; //320;
    static const int frameHeight = 180; //90; //180;
    static cv::Point2f carInFramePosition;
    static cv::Point2f carInFramePositionBirdsEye;
    cv::Point2f undefinedPoint = cv::Point2f(1000,0);
    double circleRadius = 100.0f;
// Used in camera_setup.h
    // Used in perspectiveChange()
    // Size of new perspective box
    static const int heightBirdsEyeView = 400 - cutHeight*2;
    static const int widthBirdsEyeView = 370;
    // dstPoints Box size in pixels
    // (This box is placed in the middle bottom part of the perspective box)
    int heightDstPoints;        // This are initialized in initPerspectiveVariables()
    int widthDstPoints;         // This are initialized in initPerspectiveVariables()
    static int trackLaneWidthInPixel;                        // Important
    static std::vector<cv::Point2f> srcPoints;
    static std::vector<cv::Point2f> dstPoints;
    // Perspective transform matrix
    static cv::Mat MatrixBirdsEyeView;
    static cv::Mat MatrixInverseBirdsEyeView;

    
// Used in segmentation.h
    static const int thresholdValue = 90;

// Used in line_detection.h
    // Used in fitPolinomial()
    static const int windowSize = 35;  
    static double epsilon = 9.0;  // Epsilon value for curve approximation
    // Used in findMiddle()
    // Number of points to sample the curve(High number equals more complexity)
    static const int num_points = 12;  
    // Used in are2PointsHorizontal()
    static const double slopeThreshold = 1;
    // Used in customConnectedComponentsWithThreshold()
    static const int minPixelCount = 10;                            // Defines how many pixel can make a line (removes noise)
    static const double rowThresholdCutOff = 0.4;
    // Used in removeHorizontalIf90Turn()
    static const double removeAngleMin = 70;
    static const double removeAngleMax = 110;
    // Used in getLeftRightLines()
    cv::Point2f firstPointLeftLine = undefinedPoint;
    cv::Point2f firstPointRightLine = undefinedPoint;
    cv::Point2f firstPointSingleLine = undefinedPoint;
#endif
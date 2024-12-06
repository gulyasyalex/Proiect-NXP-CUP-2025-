#ifndef CONFIG_H
#define CONFIG_H


#define ENABLE_CALIBRATE_CAMERA 0
#define ENABLE_STREAMING 0
#define ENABLE_READING_FROM_JPG 1

// Global values
    int cutHeight = 0;
    int frameWidth = 320; //160; //320;
    int frameHeight = 180; //90; //180;

// Used in camera_setup.h
    // Used in perspectiveChange()
    // Size of new perspective box
    int heightBirdsEyeView = 400 - cutHeight*2;
    int widthBirdsEyeView = 370;
    // dstPoints Box size in pixels
    // (This box is placed in the middle bottom part of the perspective box)
    int heightDstPoints;
    int widthDstPoints;
    int trackLaneWidthInPixel = 200;
    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;
    // Perspective transform matrix
    cv::Mat M;
    cv::Mat M_inv;

    
// Used in segmentation.h
    // Used in adaptiveGuassianThreshold()
    int blurBlockSize = 25;
    int AdaptGaussConstant = 2;
    int AdaptGaussBlockSize = 25;

// Used in line_detection.h
    // Used in fitPolinomial()
    int windowSize = 25;  
    double epsilon = 7.0;  // Epsilon value for curve approximation
    // Used in findMiddle()
    // Number of points to sample the curve(High number equals more complexity)
    int num_points = 15;  
    // Used in are2PointsHorizontal()
    double slopeThreshold = 0.6;
    // Used in customConnectedComponentsWithThreshold()
    double rowThresholdCutOff = 0.4;
    // Used in removeHorizontalIf90Turn()
    double removeAngleMin = 80;
    double removeAngleMax = 100;
#endif
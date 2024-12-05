#ifndef CONFIG_H
#define CONFIG_H

// Global values
    int cutHeight = 40;
    int frameWidth = 320; //160; //320;
    int frameHeight = 180; //90; //180;

// Used in camera_setup.h
    // Used in perspectiveChange()
    // Size of new perspective box
    int heightBirdsEyeView = 600 - cutHeight*2;
    int widthBirdsEyeView = 300;
    // dstPoints Box size in pixels
    // (This box is placed in the middle bottom part of the perspective box)
    int heightDstPoints = 140;
    int widthDstPoints = 200;

    
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

    double slopeThreshold = 0.6;
#endif
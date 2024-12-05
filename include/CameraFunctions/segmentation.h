#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "config.h"

cv::Mat segmentEdges(const cv::Mat& frame);
cv::Mat cropFrameTop(const cv::Mat& frame, int cutHeight);
cv::Mat resizeImage(const cv::Mat& src, int width, int height);
//cv::Mat adaptiveGuassianThreshold(const cv::Mat& frame);
//cv::Mat otsuThresholding(const cv::Mat& frame);


cv::Mat segmentEdges(const cv::Mat& gray) {

    /* Visualize
    cv::Mat outputImage = frame.clone();   
    outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
    // Display the image with lines colored
    cv::imshow("Initial", outputImage)*/;

    
    cv::Mat noiseImage;
    //noiseImage = adaptiveGuassianThreshold(gray);

    cv::threshold(gray, noiseImage, 90, 255, cv::THRESH_BINARY); //fara LED 25
    cv::bitwise_not(noiseImage, noiseImage);

    // At this point image is full of noise yet Lines are clearly visible
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(noiseImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat cleaned = cv::Mat::zeros(noiseImage.size(), CV_8UC1);

    /*// Filter and draw contours based on area
    double areaThreshold = 150.0; 
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > areaThreshold) {
            cv::drawContours(cleaned, contours, i, cv::Scalar(255), cv::FILLED);
        }
    }*/
    // Filter and draw contours based on area
    double areaThresholdMin = 300.0; 
    double areaThresholdMax = 4000.0; 
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > areaThresholdMin && area < areaThresholdMax) {
            cv::drawContours(cleaned, contours, i, cv::Scalar(255), cv::FILLED);
        }
    }
    
    // Visualize
    /*cv::Mat outputImage = cleaned.clone();
    outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
    // Display the image with lines colored
    cv::imshow("Contours", outputImage);*/

    return cleaned;
}

// Cuts pixel rows from image's top part 
cv::Mat cropFrameTop(const cv::Mat& frame, int cutHeight) {

    if (cutHeight == 0) {
        return frame;
    }

    int frameHeight = frame.rows;
    int frameWidth = frame.cols;
    
    if (cutHeight < 0 || cutHeight >= frameHeight) {
        throw std::invalid_argument("Cut Height must be between 0 and " + frameHeight);
    }

    // Create a copy of the original frame to draw the line
    /*cv::Mat frameWithLine = frame.clone();

    // Draw a horizontal line at the cutHeight
    cv::line(frameWithLine, cv::Point(0, cutHeight), cv::Point(frameWidth, cutHeight), cv::Scalar(0, 255, 255), 2);

    // Display the frame with the line
    cv::imshow("FrameWithCutLine", frameWithLine);*/

    // Define the region of interest (ROI) to crop the top part
    cv::Rect roi(0, cutHeight, frameWidth, frameHeight - cutHeight);
    cv::Mat croppedImage = frame(roi);

    return croppedImage;
}

cv::Mat resizeImage(const cv::Mat& src, int width, int height){
    
    // Resize the image to the specified width and height
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(width, height));
    return resized;
}

/*cv::Mat adaptiveGuassianThreshold(const cv::Mat& frame){
    
    // Apply Gaussian blur
    cv::Mat blur;
    cv::GaussianBlur(frame, blur, cv::Size(blurBlockSize, blurBlockSize), 0);

     // Apply adaptive threshold
    cv::Mat adaptiveThreshold;
    cv::adaptiveThreshold(blur, adaptiveThreshold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, AdaptGaussBlockSize, AdaptGaussConstant);
    
    /* Visualize
    cv::Mat outputImage = adaptiveThreshold.clone();   
    outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
    // Display the image with lines colored
    cv::imshow("AdativeGaussianThreshold", outputImage);
    
    return adaptiveThreshold;
}*/

/*// OtsuThreshold
cv::Mat otsuThresholding(const cv::Mat& frame){
    
     // Apply Gaussian blur
    cv::Mat blur;
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 2);
    cv::Mat otsuThresh;
    double otsuThreshold = cv::threshold(blur, otsuThresh, 10, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Display the results
    //std::cout << "Otsu's threshold value: " << otsuThreshold << std::endl;


    return otsuThresh;
}
*/

#endif 
#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "config.h"

cv::Mat skeletonizeFrame(const cv::Mat& thresholdedImage);
cv::Mat segmentEdges(const cv::Mat& frame);
cv::Mat cropFrameTop(const cv::Mat& frame, int cutHeight);
cv::Mat resizeImage(const cv::Mat& src, int width, int height);

// Function returns a skeletonized frame
cv::Mat skeletonizeFrame(const cv::Mat& thresholdedImage){

    cv::Mat skeleton(cv::Mat::zeros(thresholdedImage.size(), CV_8UC1));
    cv::Mat temp, eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    while (true) {
        cv::erode(thresholdedImage, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(thresholdedImage, temp, temp);
        cv::bitwise_or(skeleton, temp, skeleton);
        eroded.copyTo(thresholdedImage);

        if (cv::countNonZero(thresholdedImage) == 0) {
            break;
        }
    }
    
    return skeleton;
}
cv::Mat segmentEdges(const cv::Mat& frame) {

    cv::Mat thresholdFrame;
    cv::threshold(frame, thresholdFrame, thresholdValue, 255, cv::THRESH_BINARY);    
    cv::bitwise_not(thresholdFrame, thresholdFrame);
    cv::Mat skeleton = skeletonizeFrame(thresholdFrame);
    return skeleton;
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
#endif
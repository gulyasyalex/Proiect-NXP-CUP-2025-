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
//cv::Mat adaptiveGuassianThreshold(const cv::Mat& frame);
//cv::Mat otsuThresholding(const cv::Mat& frame);

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

    cv::threshold(frame, frame, 90, 255, cv::THRESH_BINARY); //fara LED 25
    //cv::bitwise_not(edges, edges);
    cv::Mat skeleton = skeletonizeFrame(frame);
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
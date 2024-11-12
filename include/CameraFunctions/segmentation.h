#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "config.h"

cv::Mat adaptiveGuassianThreshold(const cv::Mat& frame);
cv::Mat segmentEdges(const cv::Mat& frame);
cv::Mat cropFrameTop(const cv::Mat& frame, int cutHeight);
cv::Mat resizeImage(const cv::Mat& src, int width, int height);
int demosaic(uint16_t width, uint16_t height, const uint8_t *bayerImage, uint32_t *image);
//cv::Mat otsuThresholding(const cv::Mat& frame);


cv::Mat adaptiveGuassianThreshold(const cv::Mat& frame){
    
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
    cv::imshow("AdativeGaussianThreshold", outputImage);*/
    
    return adaptiveThreshold;
}

cv::Mat segmentEdges(const cv::Mat& frame) {

    /* Visualize
    cv::Mat outputImage = frame.clone();   
    outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
    // Display the image with lines colored
    cv::imshow("Initial", outputImage)*/;

    // Converting frame to grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat noiseImage;
    //noiseImage = adaptiveGuassianThreshold(gray);

    cv::threshold(gray, noiseImage, 80, 255, cv::THRESH_BINARY); //fara LED 25
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
    cv::Mat frameWithLine = frame.clone();

    // Draw a horizontal line at the cutHeight
    cv::line(frameWithLine, cv::Point(0, cutHeight), cv::Point(frameWidth, cutHeight), cv::Scalar(0, 255, 255), 2);

    // Display the frame with the line
    cv::imshow("FrameWithCutLine", frameWithLine);

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

int demosaic(uint16_t width, uint16_t height, const uint8_t *bayerImage, uint32_t *image)
{
    uint32_t x, y, xx, yy, r, g, b;
    uint8_t *pixel0, *pixel;

    for (y = 0; y < height; y++)
    {
        yy = y;
        if (yy == 0)
            yy++;
        else if (yy == height - 1)
            yy--;
        pixel0 = (uint8_t *)bayerImage + yy * width;
        for (x = 0; x < width; x++, image++)
        {
            xx = x;
            if (xx == 0)
                xx++;
            else if (xx == width - 1)
                xx--;
            pixel = pixel0 + xx;
            if (yy & 1)
            {
                if (xx & 1)
                {
                    r = *pixel;
                    g = (*(pixel - 1) + *(pixel + 1) + *(pixel + width) + *(pixel - width)) >> 2;
                    b = (*(pixel - width - 1) + *(pixel - width + 1) + *(pixel + width - 1) + *(pixel + width + 1)) >> 2;
                }
                else
                {
                    r = (*(pixel - 1) + *(pixel + 1)) >> 1;
                    g = *pixel;
                    b = (*(pixel - width) + *(pixel + width)) >> 1;
                }
            }
            else
            {
                if (xx & 1)
                {
                    r = (*(pixel - width) + *(pixel + width)) >> 1;
                    g = *pixel;
                    b = (*(pixel - 1) + *(pixel + 1)) >> 1;
                }
                else
                {
                    r = (*(pixel - width - 1) + *(pixel - width + 1) + *(pixel + width - 1) + *(pixel + width + 1)) >> 2;
                    g = (*(pixel - 1) + *(pixel + 1) + *(pixel + width) + *(pixel - width)) >> 2;
                    b = *pixel;
                }
            }
            *image = (b << 16) | (g << 8) | r;
        }
    }
    return 0;
}

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
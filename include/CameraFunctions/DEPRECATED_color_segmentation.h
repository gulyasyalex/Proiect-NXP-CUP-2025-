#ifndef COLOR_SEGMENTATION_H
#define COLOR_SEGMENTATION_H

#include <opencv2/opencv.hpp>

// (HSL Color Space) 
// Full range for hue since black doesn't have a specific hue
int hueHigh = 180; 
// Low lightness for black hues
int lightnessHigh = 0; 
// Low saturation for black hues
int saturationHigh = 0; 

cv::Mat global_frame;

void addThresholdSliders();
void MaskExtract(const cv::Mat& frame);
cv::Mat colorSegment(const cv::Mat& hls, cv::Scalar lowerRange, cv::Scalar upperRange);
 
// Callback functions
void onHueHighChange(int, void*) {
    hueHigh = cv::getTrackbarPos("Hue_H", "[Segment_Colour_final] mask");
    MaskExtract(global_frame);
}

void onLightnessHighChange(int, void*) {
    lightnessHigh = cv::getTrackbarPos("Lightness_H", "[Segment_Colour_final] mask");
    MaskExtract(global_frame);
}

void onSaturationHighChange(int, void*) {
    saturationHigh = cv::getTrackbarPos("Saturation_H", "[Segment_Colour_final] mask");
    MaskExtract(global_frame);
}
// Main function to extract mask and perform color segmentation
void MaskExtract(const cv::Mat& frame) {
    cv::Mat hls, mask, dst;
    global_frame = frame;

    addThresholdSliders();
    cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
    
    mask = colorSegment(hls, cv::Scalar(0, 0, 0), cv::Scalar(hueHigh, lightnessHigh, saturationHigh));
    
    //cv::Mat mask_ = mask != 0;
    //bitwise_and(frame, frame, dst, mask_);
    
    imshow("[Segment_Colour_final] mask", mask);
}

// Function to perform color segmentation
cv::Mat colorSegment(const cv::Mat& hls, cv::Scalar lowerRange, cv::Scalar upperRange) {
    cv::Mat mask;
    cv::inRange(hls, lowerRange, upperRange, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);
    return mask;
}

void addThresholdSliders() {
    static bool window_created = false;
    if (!window_created) {
        cv::namedWindow("[Segment_Colour_final] mask");

        // Create trackbars without setting the value pointer
        cv::createTrackbar("Hue_H", "[Segment_Colour_final] mask", NULL, 180, onHueHighChange);
        cv::createTrackbar("Lightness_H", "[Segment_Colour_final] mask", NULL, 255, onLightnessHighChange);
        cv::createTrackbar("Saturation_H", "[Segment_Colour_final] mask", NULL, 255, onSaturationHighChange);

        // Set initial values
        cv::setTrackbarPos("Hue_H", "[Segment_Colour_final] mask", hueHigh);
        cv::setTrackbarPos("Lightness_H", "[Segment_Colour_final] mask", lightnessHigh);
        cv::setTrackbarPos("Saturation_H", "[Segment_Colour_final] mask", saturationHigh);

        window_created = true;
    }
}
#endif 
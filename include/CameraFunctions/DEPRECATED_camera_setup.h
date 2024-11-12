#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include <opencv2/opencv.hpp>
#include "include/config.h"

cv::Mat refineEdgesPerspective(const cv::Mat& frame) {
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1); 

        for (int i = 0; i < frame.rows; i++) {
            for (int j = 0; j < frame.cols; j++) {
                uchar pixelValue = frame.at<uchar>(i, j);
                if (pixelValue > 0) {
                    mask.at<uchar>(i, j) = 255; 
                }
            }
        }

        // Visualize the mask using imshow
        //cv::imshow("Pixels with value > 100", mask);
        return mask;
}
/**
 * @brief Inverse Perspective Mapping(IPM)
 * 
 * This function changes the perspective of the road for better edge-detection
 * 
 * @param frame The original image on which lanes will be detected and highlighted.
 *              Should be a multi-channel image (e.g., BGR).
 * @param cutHeight The perspective will adapt based on the pixel rows that were cut in order to keep image stable
 */
cv::Mat perspectiveChange(const cv::Mat& frame, float cutHeight){

    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(160, 180 - cutHeight),  // Example top-left corner
        cv::Point2f(460, 190 - cutHeight), // Example top-right corner
        cv::Point2f(40, 350 - cutHeight), // Example bottom-left corner
        cv::Point2f(580, 350 - cutHeight) // Example bottom-right corner
    };

    // Destination points for the bird's-eye view
    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 - static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f - static_cast<float>(heightDstPoints) 
        ), // Top-left corner
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 + static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f - static_cast<float>(heightDstPoints) 
        ), // Top-right corner
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 - static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f 
        ), // Bottom-left corner
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 + static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f 
        ) // Bottom-right corner
    };

    // Compute the perspective transform matrix
    cv::Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transform
    cv::Mat birdEyeView;
    cv::warpPerspective(frame, birdEyeView, M, cv::Size(widthBirdsEyeView, heightBirdsEyeView));

    // Make distorted lines, caused by perpectiveWrap, white again
    birdEyeView = refineEdgesPerspective(birdEyeView);

    // Compute the bounding box of the destination points
    cv::Rect boundingBox = cv::boundingRect(dstPoints);

    // Crop the transformed image to the bounding box
    cv::Mat croppedBirdEyeView = birdEyeView(boundingBox);

    // Display the cropped image
    //cv::imshow("Cropped Bird-Eye View", croppedBirdEyeView);
    
    
    cv::Mat imageWithPoints;
    cvtColor(frame.clone(), imageWithPoints, cv::COLOR_GRAY2BGR);
    for (const auto& point : srcPoints) {
        cv::circle(imageWithPoints, point, 5, cv::Scalar(0, 0, 255), -1); // Red color, filled circle with radius 10
    }
    cv::imshow("imageWithPoints", imageWithPoints);
  
    cv::Mat birdEyeViewWithPoints;
    cvtColor(birdEyeView.clone(), birdEyeViewWithPoints, cv::COLOR_GRAY2BGR);
    // Draw circles at each destination point
    for (const auto& point : dstPoints) {
        cv::circle(birdEyeViewWithPoints, point, 5, cv::Scalar(0, 0, 255), -1); // Red color, filled circle with radius 10
    }    
    cv::imshow("birdEyeViewWithPoints", birdEyeViewWithPoints);

    return birdEyeView;
}

#endif 
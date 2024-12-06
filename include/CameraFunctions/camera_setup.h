#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "config.h"
#include <unistd.h>
#include <iostream>
#include <vector>
#include <fstream>


void printWorkingDirectory();
std::string getCameraIndex();
cv::Mat refineEdgesPerspective(const cv::Mat& frame);
void initPerspectiveVariables();
cv::Mat perspectiveChange(const cv::Mat& frame, float cutHeight);
void writePointsToTxt(const std::vector<cv::Point>& points, const std::string& filename);
std::vector<cv::Point> readPointsFromTxt(const std::string& filename);

void printWorkingDirectory() {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "Current Working Directory: " << cwd << std::endl;
    } else {
        std::cerr << "Error getting current working directory." << std::endl;
    }
}
// Function to execute the shell script and capture the output
std::string getCameraIndex() {
    // Path to the shell script
    std::string scriptPath = "./include/CameraFunctions/find_camera_index.sh";
    char buffer[128];
    std::string result = "";

    // Open a pipe to run the shell script
    FILE* pipe = popen(scriptPath.c_str(), "r");
    if (!pipe) {
        std::cerr << "Error: Unable to open pipe to script." << std::endl;
        return "-1";
    }

    // Read the output of the script
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    // Close the pipe
    pclose(pipe);

    // Remove any trailing newline character
    result.erase(result.find_last_not_of(" \n\r\t") + 1);

    return result;
}

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
void initPerspectiveVariables(){
    auto loadedPoints = readPointsFromTxt("interpolated_points.txt");

    // Display loaded points
    std::cout << "Loaded Points:\n";
    for (const auto& point : loadedPoints) {
        std::cout << "(" << point.x << ", " << point.y << ")\n";
    }
    srcPoints = {
        cv::Point2f(loadedPoints[0].x, loadedPoints[0].y - cutHeight),  // Example top-left corner
        cv::Point2f(loadedPoints[1].x, loadedPoints[1].y - cutHeight), // Example bottom-left corner
        cv::Point2f(loadedPoints[2].x, loadedPoints[2].y - cutHeight), // Example top-right corner
        cv::Point2f(loadedPoints[3].x, loadedPoints[3].y - cutHeight) // Example bottom-right corner
    };
    widthDstPoints = 200;
    heightDstPoints = 160;

    // Destination points for the bird's-eye view
    dstPoints = {
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 - static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f - static_cast<float>(heightDstPoints) 
        ), // Top-left corner
        cv::Point2f(
        static_cast<float>(widthBirdsEyeView) / 2 - static_cast<float>(widthDstPoints) / 2,
        static_cast<float>(heightBirdsEyeView) - 10.0f 
        ), // Bottom-left corner
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 + static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f - static_cast<float>(heightDstPoints) 
        ), // Top-right corner
        cv::Point2f(
            static_cast<float>(widthBirdsEyeView) / 2 + static_cast<float>(widthDstPoints) / 2,
            static_cast<float>(heightBirdsEyeView) - 10.0f 
        ) // Bottom-right corner
    };

    M = cv::getPerspectiveTransform(srcPoints, dstPoints);
    M_inv = M.inv();
}
cv::Mat perspectiveChange(const cv::Mat& frame){

    if(srcPoints.empty()){
        initPerspectiveVariables();
    }
    
    // Compute the perspective transform matrix
    cv::Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transform
    cv::Mat birdEyeView;
    cv::warpPerspective(frame, birdEyeView, M, cv::Size(widthBirdsEyeView, heightBirdsEyeView));

    // Make distorted lines, caused by perpectiveWrap, white again
    //birdEyeView = refineEdgesPerspective(birdEyeView);

    // Compute the bounding box of the destination points
    cv::Rect boundingBox = cv::boundingRect(dstPoints);

    // Crop the transformed image to the bounding box
    cv::Mat croppedBirdEyeView = birdEyeView(boundingBox);

    // Display the cropped image
    //cv::imshow("Cropped Bird-Eye View", croppedBirdEyeView);
    
    
    cv::Mat imageWithPoints = frame.clone();
    for (const auto& point : srcPoints) {
        cv::circle(imageWithPoints, point, 5, cv::Scalar(0, 0, 255), -1); // Red color, filled circle with radius 10
    }
    //cv::imshow("imageWithPoints", imageWithPoints);
  
    cv::Mat birdEyeViewWithPoints = birdEyeView.clone();
    // Draw circles at each destination point
    for (const auto& point : dstPoints) {
        cv::circle(birdEyeViewWithPoints, point, 5, cv::Scalar(0, 0, 255), -1); // Red color, filled circle with radius 10
    }    
    //cv::imshow("birdEyeViewWithPoints", birdEyeViewWithPoints);

    return birdEyeViewWithPoints;
}

void perspectiveChangeLineM(std::vector<cv::Point>& line) {
    if (M.empty()) {
        initPerspectiveVariables();
    }

    // Convert input points to float
    std::vector<cv::Point2f> floatLine(line.begin(), line.end());

    // Transform the points
    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(floatLine, transformedPoints, M);

    // Convert transformed points back to integer if needed
    line.clear();
    for (const auto& pt : transformedPoints) {
        line.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
}

void perspectiveChangeLineMinv(std::vector<cv::Point>& line) {
    if (M_inv.empty()) {
        initPerspectiveVariables();
    }

    // Convert input points to float
    std::vector<cv::Point2f> floatLine(line.begin(), line.end());

    // Transform the points
    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(floatLine, transformedPoints, M_inv);

    // Convert transformed points back to integer if needed
    line.clear();
    for (const auto& pt : transformedPoints) {
        line.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
}


// Function to write points to a TXT file
void writePointsToTxt(const std::vector<cv::Point>& points, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& point : points) {
            file << point.x << " " << point.y << "\n"; // Save as "x y" per line
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for writing: " << filename << "\n";
    }
}

// Function to read points from a TXT file
std::vector<cv::Point> readPointsFromTxt(const std::string& filename) {
    std::vector<cv::Point> points;
    std::ifstream file(filename);
    if (file.is_open()) {
        int x, y;
        while (file >> x >> y) { // Read "x y" from each line
            points.emplace_back(x, y);
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for reading: " << filename << "\n";
    }
    return points;
}

void saveImage(const std::string& filename, const cv::Mat& frame){
    bool success = cv::imwrite(filename, frame);

    if (success) {
        std::cout << "Image successfully saved to " << std::endl;
    } else {
        std::cerr << "Failed to save the image!" << std::endl;
    }
}

#endif 
#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include "config.h"

double lineLength(const cv::Vec4i& line);
double euclideanDistance(cv::Point2f p1, cv::Point2f p2);
double euclideanDistanceCoord(int x1, int y1, int x2, int y2);
double calculateSignedAngle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
double mapAngleToServo(double angle);
cv::Point2f calculateMidpoint(const cv::Point2f& p1, const cv::Point2f& p2);
cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& line);
std::vector<cv::Point2f> generateNeighborhood(int radius);

// Function to calculate the length of a line
double lineLength(const cv::Vec4i& line) {
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// Function to calculate the Euclidean distance between two OpenCv points
double euclideanDistance(cv::Point2f p1, cv::Point2f p2) {
   return std::hypot(p1.x - p2.x, p1.y - p2.y);
}
double euclideanDistance2f(cv::Point2f p1, cv::Point2f p2) {
   return std::hypot(p1.x - p2.x, p1.y - p2.y);
}
// Function to calculate the Euclidean distance between two points in coordinate values
double euclideanDistanceCoord(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

// Function to calculate the signed angle between three points 
double calculateSignedAngle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3)
{
    cv::Point2f vec1 = p1 - p2; // Vector de la p2 la p1
    cv::Point2f vec2 = p3 - p2; // Vector de la p2 la p3
    double dotProduct = vec1.x * vec2.x + vec1.y * vec2.y;
    double crossProduct = vec1.x * vec2.y - vec1.y * vec2.x;
    double angle = std::atan2(crossProduct, dotProduct) * 180.0 / CV_PI;
    return angle;
}

/*
Protractor Degrees show:
    - MAX LEFT: 140 degrees
    - MIDDLE: 90 degrees
    - MAX RIGHT: 40 degrees
Translated it should be (-50,0,50) 
Servo Shows:
    - MAX LEFT: -30 degrees
    - MIDDLE: 0 degrees
    - MAX RIGHT: 30 degrees
Linear ecuasion should be written as follows:

    ServoAngle = m(slope) * ProtractorAngle + b(intercept);

2 Points: (S = 30; P = 50) and (S = 0; P = 0)
    m = (30 - 0)/ (50 - 0) = 3/5

Then for point (S = 0; P = 0)
    0 = 3/5 * 0 + b;  => b = 0;

So any ServoAngle = 3/5 * ProtractorAngle;


*/
double mapAngleToServo(double angle)
{
    double mappedAngle = 3.0/5.0 * angle;
    if (mappedAngle < maxLeftServoAngle)
        mappedAngle = maxLeftServoAngle;
    else if (mappedAngle > maxRightServoAngle)
        mappedAngle = maxRightServoAngle;

    return mappedAngle;
}

// Function to calculate the midpoint between two points
cv::Point2f calculateMidpoint(const cv::Point2f& p1, const cv::Point2f& p2) {
    return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
// Function to calculate the centroid of a line (average of the points)
cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& line) {
    cv::Point2f centroid(0, 0);
    for (const auto& point : line) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= line.size();
    centroid.y /= line.size();
    return centroid;
}
// Function that generates a radius of values for customConnectedComponentsWithThreshold
std::vector<cv::Point2f> generateNeighborhood(int radius) {
    std::vector<cv::Point2f> neighbors;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (std::sqrt(dx * dx + dy * dy) <= radius) { // Ensure point lies within the circle
                neighbors.emplace_back(dx, dy);
            }
        }
    }
    return neighbors;
}

#endif
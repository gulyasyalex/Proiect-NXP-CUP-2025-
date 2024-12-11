#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include "config.h"

double lineLength(const cv::Vec4i& line);
double euclideanDistance(cv::Point2f p1, cv::Point2f p2);
double euclideanDistanceCoord(int x1, int y1, int x2, int y2);
double calculateAngle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
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
// Function to calculate the angle between three points used in removeHorizontalIf90Turn
double calculateAngle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
    cv::Point2f vec1 = p1 - p2;
    cv::Point2f vec2 = p3 - p2;
    double dotProduct = vec1.x * vec2.x + vec1.y * vec2.y;
    double mag1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
    double mag2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    return std::acos(dotProduct / (mag1 * mag2)) * 180.0 / CV_PI;
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
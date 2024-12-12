#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "math_functions.h"


cv::Point2f findHighestIntersection(const std::vector<cv::Point2f>& curve, const cv::Point2f& circleCenter, double circleRadius) {
    std::vector<cv::Point2f> intersections;

    for (size_t i = 0; i < curve.size() - 1; ++i) {
        cv::Point2f p1 = curve[i];
        cv::Point2f p2 = curve[i + 1];
        cv::Point2f segmentVector = p2 - p1;

        double dx = segmentVector.x;
        double dy = segmentVector.y;

        // Coefficients for quadratic equation a*t^2 + b*t + c = 0
        double a = dx * dx + dy * dy;
        double b = 2 * (dx * (p1.x - circleCenter.x) + dy * (p1.y - circleCenter.y));
        double c = (p1.x - circleCenter.x) * (p1.x - circleCenter.x) +
                   (p1.y - circleCenter.y) * (p1.y - circleCenter.y) - 
                   circleRadius * circleRadius;

        // Compute discriminant
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            continue; // No intersection
        }

        // Solve quadratic equation for t
        double sqrtDiscriminant = std::sqrt(discriminant);
        for (int sign : {-1, 1}) {
            double t = (-b + sign * sqrtDiscriminant) / (2 * a);

            // Check if t is within [0, 1]
            if (t >= 0.0 && t <= 1.0) {
                cv::Point2f intersection = p1 + t * segmentVector;
                intersections.push_back(intersection);
            }
        }
    }

    // If no intersections found, return empty vector
    if (intersections.empty()) {
        return undefinedPoint;
    }

    cv::Point2f lowestYPoint = intersections[0];

    for (const auto& point : intersections) {
        if (point.y < lowestYPoint.y) {
            lowestYPoint = point;
        }
    }

    return lowestYPoint;
}

// Function to calculate the shortest distance from a point to a polyline
double shortestDistanceToCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point, double circleRadius) {
    double minDistance = std::numeric_limits<double>::max();
    double maxDistance = std::numeric_limits<double>::min();
    double tolerance = 1; // Adding 1px tolerance 

    for (size_t i = 0; i < curve.size() - 1; ++i) {
        cv::Point2f p1 = curve[i];
        cv::Point2f p2 = curve[i + 1];

        // Vector from p1 to p2
        cv::Point2f lineVec = p2 - p1;
        // Vector from p1 to the given point
        cv::Point2f pointVec = point - p1;

        // Project pointVec onto lineVec, normalize by the length squared of lineVec
        float t = lineVec.dot(pointVec) / lineVec.dot(lineVec);

        cv::Point2f closestPoint;
        if (t < 0.0f) {
            // Closest point is p1
            closestPoint = p1;
        } else if (t > 1.0f) {
            // Closest point is p2
            closestPoint = p2;
        } else {
            // Closest point is within the segment
            closestPoint = p1 + t * lineVec;
        }

        // Compute Euclidean distance using your function
        double dist = euclideanDistance(point, closestPoint);
        minDistance = std::min(minDistance, dist);
        maxDistance = std::max(maxDistance, dist);
    }

    maxDistance = std::min(maxDistance, circleRadius);
    minDistance = std::max(minDistance, maxDistance);

    minDistance = minDistance + tolerance;
    return minDistance;
}

void radiusIncrease(double& radius){

    static bool direction = true;

    if(radius >= 350)
        direction = false;
    else if (radius <= 0)
        direction = true;    
    
    if(direction)
        radius++;
    else if(!direction)
        radius--;
}

void pointMoveAcrossFrame(cv::Point2f& point, cv::Point2f& topPoint){

    static bool direction = true;

    if(point.x >= birdsEyeViewWidth)
        direction = false;
    else if (point.x < 0)
        direction = true;

    if(direction)
    {
        point.x++;
        topPoint.x = point.x;
    }
    else if(!direction)
    {
        point.x--;
        topPoint.x = point.x;
    }
}

#endif
#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "math_functions.h"

cv::Point2f findHighestIntersection(const cv::Point2f& circleCenter, double circleRadius,const std::vector<cv::Point2f>& curve);

cv::Point2f findHighestIntersection(
    const cv::Point2f& circleCenter, double circleRadius, const std::vector<cv::Point2f>& curve) {
    std::vector<cv::Point2f> intersections;

    double tolerance = 1e-4; // Increased tolerance for floating-point precision

    for (size_t i = 0; i < curve.size() - 1; ++i) {
        cv::Point2f p1 = curve[i];
        cv::Point2f p2 = curve[i + 1];

        // Skip degenerate segments
        if (euclideanDistance2f(p1, p2) < tolerance) {
            continue;
        }

        cv::Point2f segmentVector = p2 - p1;
        cv::Point2f centerToP1 = circleCenter - p1;

        double segmentLengthSquared = segmentVector.x * segmentVector.x + segmentVector.y * segmentVector.y;

        double t = (centerToP1.x * segmentVector.x + centerToP1.y * segmentVector.y) / segmentLengthSquared;
        t = std::max(0.0, std::min(1.0, t)); // Clamp t to [0, 1]

        cv::Point2f closestPoint = p1 + t * segmentVector;

        double distanceToClosestPoint = euclideanDistance2f(circleCenter, closestPoint);

        if (distanceToClosestPoint <= circleRadius + tolerance) {
            double offsetDistance = std::sqrt(circleRadius * circleRadius - distanceToClosestPoint * distanceToClosestPoint);
            cv::Point2f unitVector = segmentVector / std::sqrt(segmentLengthSquared);

            // Calculate intersection points
            cv::Point2f intersection1 = closestPoint + offsetDistance * unitVector;
            cv::Point2f intersection2 = closestPoint - offsetDistance * unitVector;

            // Validate that intersections lie on the segment and within the circle
            auto isValidIntersection = [&](const cv::Point2f& point) {
                return (std::min(p1.x, p2.x) - tolerance <= point.x && point.x <= std::max(p1.x, p2.x) + tolerance) &&
                    (std::min(p1.y, p2.y) - tolerance <= point.y && point.y <= std::max(p1.y, p2.y) + tolerance) &&
                    (std::pow(point.x - circleCenter.x, 2) + std::pow(point.y - circleCenter.y, 2) <= std::pow(circleRadius, 2) + tolerance);
            };

            intersections.push_back(intersection1);
            intersections.push_back(intersection2);
        }
    }

    // Filter intersections to ensure they lie within the circle's boundary
    intersections.erase(
        std::remove_if(intersections.begin(), intersections.end(), [&](const cv::Point2f& pt) {
            double dx = pt.x - circleCenter.x;
            double dy = pt.y - circleCenter.y;
            return (dx * dx + dy * dy > circleRadius * circleRadius + tolerance);
        }),
        intersections.end()
    );

     if (intersections.empty()) {
        // Return a default value to keep straight steering
        return cv::Point2f(widthBirdsEyeView, heightBirdsEyeView - 20);
    }

    cv::Point2f lowestYPoint = intersections[0];

    for (const auto& point : intersections) {
        if (point.y < lowestYPoint.y) {
            lowestYPoint = point;
        }
    }

    return lowestYPoint;

}




#endif
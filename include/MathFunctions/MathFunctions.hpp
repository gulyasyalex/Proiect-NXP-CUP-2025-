#ifndef MATH_FUNCTIONS_HPP
#define MATH_FUNCTIONS_HPP

#include <opencv2/opencv.hpp>
#include <cmath>
#include "config.h"

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>

double lineLength(const cv::Vec4i& line);
double euclideanDistance(cv::Point2f p1, cv::Point2f p2);
double euclideanDistanceCoord(int x1, int y1, int x2, int y2);
double calculateSignedAngleThreePoints(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
double mapAngleToServo(double angle);
cv::Point2f calculateMidpoint(const cv::Point2f& p1, const cv::Point2f& p2);
cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& line);
std::vector<cv::Point2f> generateNeighborhood(int radius);
std::vector<cv::Point2f> closestSegmentOnCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point);
double computeAngleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2);
double roundToTwoDecimals(double value);
std::string to_string_with_precision(double value, int precision);
int doubleCmp(double num1, double num2);
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
float perpendicularDistance(const cv::Point2f& p, const cv::Point2f& start, const cv::Point2f& end);
void rdpSimplify(const std::vector<cv::Point2f>& input, double epsilon, std::vector<cv::Point2f>& output);


#endif
#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "mathFunctions.h"
int floatCmp(float num1, float num2) {
	if (fabsf(num1 - num2) < FLT_EPSILON) {
		return 0;
	}
	else if (num1 > num2) {
		return 1;
	}
	return -1;
}
static float carTurnMaxSpeed(float _turn_radius, float _friction_coefficient, float _downward_acceleration) {
	float _max_speed;
	_max_speed = sqrtf(fabs(_friction_coefficient * _turn_radius * _downward_acceleration));
	return _max_speed;
}
static float RearWheelTurnRadius(float wheelBase, float turnAngle) {
	float angle;
	//float temp_sin = sinf(turnAngle);
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	float temp_sin = tanf(turnAngle);
	if (floatCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	//angle = (wheelBase / tanf(turnAngle));
	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}
static float CalculateCarSpeed(float _min_speed, float _max_speed, float _wheel_base, float _friction_coefficient, float _downward_acceleration, float _turn_angle) {
	float new_car_speed, turn_radius;
	turn_radius = RearWheelTurnRadius(_wheel_base, _turn_angle);
	if (floatCmp(turn_radius, 0.0f) < 0) {
		new_car_speed = _max_speed;
	}
	else{
		new_car_speed = carTurnMaxSpeed(turn_radius, _friction_coefficient, _downward_acceleration);
	}
	
	new_car_speed = MAX(_min_speed, new_car_speed);
	new_car_speed = MIN(_max_speed, new_car_speed);

	return new_car_speed;
}


class PurePursuit {
private:
    double trackCurvatureRadius;
    double speed = 0;
    double lookAheadDistanceInCm;
    double steeringAngleDegrees;
    double steeringAngleServo;
    cv::Point2f lookAheadPoint;
    double angleHeadingTarget;
    SharedConfig* config;

public:
    PurePursuit();
    void setParameters(std::shared_ptr<SharedConfig> global_config);
    void computePurePursuit(std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,
                                double pixelSizeInCm, cv::Point2f carTopPoint);
    double adjustSpeed(double trackCurvatureRadius, double currentSpeed);
    double computeLookAheadDistance(double trackCurvatureRadius, std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,double pixelSizeInCm);
    cv::Point2f findHighestIntersection(const std::vector<cv::Point2f>& curve, 
                                    const cv::Point2f& circleCenter, double circleRadius); 
                                    
    double computeK(double R);                 
    double calculateServoValue(double angleRadians, double lookaheadDistance);
    double shortestDistanceToCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point);
    void radiusIncrease(double& radius);
    void pointMoveAcrossFrame(cv::Point2f& point, cv::Point2f& topPoint);
    double computeCurvatureRadius(const cv::Point2f &A,
                                    const cv::Point2f &B,
                                    const cv::Point2f &C);
                                    
    int findClosestIndex(const std::vector<cv::Point2f> &points, const cv::Point2f &carPos);
    /*double computeCurvatureRadiusInFrontOfCar(const std::vector<cv::Point2f> &midLine,
                                   const cv::Point2f &carPos);
    */   
    double computeCurvatureRadiusInFrontOfCar(const std::vector<cv::Point2f> &midLine,
                                                       const cv::Point2f &carPos,
                                                       double lookAheadDistance);
    
    double getTrackCurvatureRadius(){
        return this->trackCurvatureRadius;
    }
    void setSpeed(double speed){
        this->speed = speed;
    }
    double getSpeed(){
        return this->speed;
    }
    void setLookAheadDistanceInCm(double lookAheadDistanceInCm){
        this->lookAheadDistanceInCm = lookAheadDistanceInCm;
    }
    double getLookAheadDistanceInCm(){
        return this->lookAheadDistanceInCm;
    }
    double getSteeringAngleServo(){
        return this->steeringAngleServo;
    }
    double getSteeringAngleDegrees(){
        return this->steeringAngleDegrees;
    }
    cv::Point2f getLookAheadPoint(){
        return this->lookAheadPoint;
    }
};
PurePursuit::PurePursuit(){}
void PurePursuit::setParameters(std::shared_ptr<SharedConfig> global_config){
        this->config = global_config.get();
    }
void PurePursuit::computePurePursuit(std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,
                                double pixelSizeInCm, cv::Point2f carTopPoint){
    // Track Curvature is used in speed and lookahead distance formulas
    trackCurvatureRadius = computeCurvatureRadiusInFrontOfCar(allMidPoints, carInFramePositionBirdsEye, ((lookAheadDistanceInCm+30)*pixelSizeInCm));
    
    //speed = adjustSpeed(trackCurvatureRadius, speed);
    lookAheadDistanceInCm = computeLookAheadDistance(trackCurvatureRadius, allMidPoints,
                                         carInFramePositionBirdsEye, pixelSizeInCm);
    // std::cout << "lookAheadDistanceInCm:" << lookAheadDistanceInCm << std::endl;
    lookAheadPoint = findHighestIntersection(allMidPoints, carInFramePositionBirdsEye, lookAheadDistanceInCm/pixelSizeInCm);  
    angleHeadingTarget = calculateSignedAngle(carTopPoint, carInFramePositionBirdsEye, lookAheadPoint);
    
    speed = CalculateCarSpeed(config->minSpeed, config->maxSpeed, wheelBaseInCm, 0.4, 981, angleHeadingTarget);
    std::cout << "(Constantin) speed:" << speed << "\n";

    steeringAngleServo = calculateServoValue(angleHeadingTarget, lookAheadDistanceInCm);                 
}

double PurePursuit::adjustSpeed(double trackCurvatureRadius, double currentSpeed) {
    
    double adjustedSpeed = config->maxSpeed * (1 - config->curvatureFactor / trackCurvatureRadius);
    adjustedSpeed = std::max(config->minSpeed, std::min(config->maxSpeed, adjustedSpeed));
    std::cout << "adjustedSpeed: " << adjustedSpeed << "\n";
    // Applying smoothing for stability
    double alpha = 0.1; // Smoothing factor
    double speed = currentSpeed * (1 - alpha) + adjustedSpeed * alpha;

    std::cout << "adjustedSpeed after alpha: " << speed << "\n";
    return speed;

}


double PurePursuit::computeLookAheadDistance(double trackCurvatureRadius, std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye, double pixelSizeInCm) {

    // std::cout << "trackCurvatureRadius before K: " << trackCurvatureRadius << "\n";
    
    static double k = computeK(trackCurvatureRadius);  // Initialize once
    k = 0.8 * k + 0.2 * computeK(trackCurvatureRadius);  // Update with smoothing

    // std::cout << "Computed K: " << k << "\n";
    double minimumDistance = pixelSizeInCm * shortestDistanceToCurve(allMidPoints, carInFramePositionBirdsEye);

    double lookAheadDistance = k * speed + minimumDistance;
    lookAheadDistance = std::max(config->minLookAheadInCm, std::min(config->maxLookAheadInCm, lookAheadDistance));

    return lookAheadDistance;
}

cv::Point2f PurePursuit::findHighestIntersection(const std::vector<cv::Point2f>& curve, const cv::Point2f& circleCenter, double circleRadius) {
    
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

double PurePursuit::computeK(double R)
{
    // std::cout << "Computed K R Parameter: " << R << "\n";
    if (R <= config->R_minInCm) return config->k_min;
    if (R >= config->R_maxInCm) return config->k_max;

    // Linear interpolation between k_min and k_max
    double ratio = (R - config->R_minInCm) / (config->R_maxInCm - config->R_minInCm);
    // std::cout << "Compute K Ratio: " << ratio << "\n";
    return config->k_min + ratio * (config->k_max - config->k_min);
}

// Function to calculate servo value
double PurePursuit::calculateServoValue(double angleRadians, double lookaheadDistance) {

    double steeringAngleRadians = atan((2.0 * wheelBaseInCm * sin(angleRadians)) / lookaheadDistance);
    this->steeringAngleDegrees = steeringAngleRadians * 180.0 / CV_PI;
    double servoValue = (this->steeringAngleDegrees / maxSteeringAngleDegrees) * maxServoAngle * config->servoTurnAdjustmentCoefficient;

    // Clamp servo value to [-30, 30]
    servoValue = std::max(maxLeftServoAngle, std::min(maxRightServoAngle, servoValue));

    return servoValue;
}


// Function to calculate the shortest distance from a point to a polyline
double PurePursuit::shortestDistanceToCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point) {
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
    }

    minDistance = minDistance + tolerance;
    return minDistance;
}

void PurePursuit::radiusIncrease(double& radius){

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

void PurePursuit::pointMoveAcrossFrame(cv::Point2f& point, cv::Point2f& topPoint){

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

double PurePursuit::computeCurvatureRadius(const cv::Point2f &A,
                              const cv::Point2f &B,
                              const cv::Point2f &C)
{
    double a = euclideanDistance(B, C);
    double b = euclideanDistance(C, A);
    double c = euclideanDistance(A, B);

    double s = 0.5 * (a + b + c);

    double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

    if (area < 1e-9) {
        return 1e9;
    }

    double radius = (a * b * c) / (4.0 * area);
    return radius;
}


int PurePursuit::findClosestIndex(const std::vector<cv::Point2f> &points,
                     const cv::Point2f &carPos)
{
    int closestIdx = -1;
    double minDist = std::numeric_limits<double>::max();

    for (int i = 0; i < (int)points.size(); i++) {
        double dist = euclideanDistance(points[i], carPos);
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }
    return closestIdx;
}

double PurePursuit::computeCurvatureRadiusInFrontOfCar(const std::vector<cv::Point2f> &midLine,
                                                       const cv::Point2f &carPos,
                                                       double lookAheadDistance) {
    if (midLine.size() < 3) {
        return 1e9; // Treat as nearly straight
    }

    int iClosest = findClosestIndex(midLine, carPos);
    if (iClosest < 0) {
        return 1e9;
    }

    double minRadius = std::numeric_limits<double>::max();

    // Search forward until we exceed lookAheadDistance
    for (int i = iClosest; i < (int)midLine.size() - 2; i++) {
        double dist = cv::norm(midLine[i] - carPos);
        if (dist > lookAheadDistance) break; // Stop once we exceed lookahead

        const cv::Point2f &A = midLine[i];
        const cv::Point2f &B = midLine[i + 1];
        const cv::Point2f &C = midLine[i + 2];

        double R = computeCurvatureRadius(A, B, C);
        minRadius = std::min(minRadius, R);
    }

    return minRadius;
}

/*
double PurePursuit::computeCurvatureRadiusInFrontOfCar(const std::vector<cv::Point2f> &midLine,
                                                      const cv::Point2f &carPos)
{
    if (midLine.size() < 3) {
        return 1e9; // treat as near-infinite (straight)
    }

    int iClosest = findClosestIndex(midLine, carPos);
    if (iClosest < 0) {
        return 1e9;
    }

    double minRadius = std::numeric_limits<double>::max();

    // Sliding window: calculate curvature for multiple triplets
    for (int i = iClosest; i <= (int)midLine.size() - 3; i++) {
        const cv::Point2f &A = midLine[i];
        const cv::Point2f &B = midLine[i + 1];
        const cv::Point2f &C = midLine[i + 2];
        double R = computeCurvatureRadius(A, B, C);

        if (R < minRadius) {
            minRadius = R;
        }
    }

    return minRadius;
}
*/
#endif
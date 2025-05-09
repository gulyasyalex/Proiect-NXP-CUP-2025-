#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "MathFunctions/MathFunctions.hpp"
#include "config.h"

class PurePursuit {
private:
    double trackCurvatureRadius;
    double speed = 0;
    double lookAheadDistanceInCm = 0;
    double steeringAngleDegrees;
    double steeringAngleServo;
    cv::Point2f lookAheadPoint;
    double angleHeadingTarget;
    bool startSpeedOptimizedForTorque;
    SharedConfig* config;

public:
    PurePursuit();
    void setParameters(std::shared_ptr<SharedConfig> global_config);
    void computePurePursuit(std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,
                                double pixelSizeInCm, cv::Point2f carTopPoint, bool isFinishLineDetected);
    double adjustSpeed(double trackCurvatureRadius, double currentSpeed);
    /*double computeLookAheadDistance(double trackCurvatureRadius, std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,double pixelSizeInCm, double speed);*/
    double computeLookAheadDistance(std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye, cv::Point2f carTopPoint, double pixelSizeInCm);
    cv::Point2f findHighestIntersection(const std::vector<cv::Point2f>& curve, 
                                    const cv::Point2f& circleCenter, double circleRadius); 
                                    
    //double computeK(double R);                 
    double computeK(double angle);                 
    double calculateServoValue(double angleRadians, double lookaheadDistance);
    double shortestDistanceToCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point);
    double longestDistanceOnCurveFromPoint(const std::vector<cv::Point2f>& curve, const cv::Point2f& point);
    void radiusIncrease(double& radius);
    void pointMoveAcrossFrame(cv::Point2f& point, cv::Point2f& topPoint);
    double computeCurvatureRadius(const cv::Point2f &A,
                                    const cv::Point2f &B,
                                    const cv::Point2f &C);
                                    
    int findClosestIndex(const std::vector<cv::Point2f> &points, const cv::Point2f &carPos);
    double computeCurvatureRadiusInFrontOfCar(const std::vector<cv::Point2f> &midLine,
                                                       const cv::Point2f &carPos);
    double carBoostExitCornerSpeed(double new_car_speed, double _turn_angle);
    double carTurnMaxSpeed(double _turn_radius, double _friction_coefficient, double _downward_acceleration);
    double RearWheelTurnRadius(double wheelBase, double turnAngle);
    double CalculateCarSpeed(double _min_speed, double _max_speed, double _wheel_base, double _friction_coefficient, double _downward_acceleration, double _turn_angle);

    double getTrackCurvatureRadius(){
        return this->trackCurvatureRadius;
    }
    bool getStartSpeedOptimizedForTorque(double startSpeedOptimizedForTorque){
        return this->startSpeedOptimizedForTorque;
    }
    void setStartSpeedOptimizedForTorque(double startSpeedOptimizedForTorque){
        this->startSpeedOptimizedForTorque = startSpeedOptimizedForTorque;
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
#endif
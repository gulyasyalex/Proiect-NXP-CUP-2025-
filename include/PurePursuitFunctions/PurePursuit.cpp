#include "PurePursuitFunctions/PurePursuit.hpp"

PurePursuit::PurePursuit(){}
void PurePursuit::setParameters(std::shared_ptr<SharedConfig> global_config){
        this->config = global_config.get();
    }
void PurePursuit::computePurePursuit(std::vector<cv::Point2f> allMidPoints,
                                cv::Point2f carInFramePositionBirdsEye,
                                double pixelSizeInCm, cv::Point2f carTopPoint, bool isFinishLineDetected)
{
    // Track Curvature is used in speed and lookahead distance formulas
    //std::cout << "allMidPoints: " << allMidPoints << " \n";
    this->trackCurvatureRadius = computeCurvatureRadiusInFrontOfCar(allMidPoints, carInFramePositionBirdsEye);
    
    //speed = adjustSpeed(trackCurvatureRadius, speed);
    this->lookAheadDistanceInCm = computeLookAheadDistance(trackCurvatureRadius, allMidPoints,
                                         carInFramePositionBirdsEye, pixelSizeInCm, this->speed);
    // std::cout << "lookAheadDistanceInCm:" << lookAheadDistanceInCm << std::endl;
    this->lookAheadPoint = findHighestIntersection(allMidPoints, carInFramePositionBirdsEye, lookAheadDistanceInCm/pixelSizeInCm);  
    this->angleHeadingTarget = calculateSignedAngle(carTopPoint, carInFramePositionBirdsEye, this->lookAheadPoint);
    
    if(startSpeedOptimizedForTorque)
    {
        this->speed = DEFAULT_TORQUE_SPEED;
    }
    else
    {
        if(isFinishLineDetected)
        {
            this->speed = CalculateCarSpeed(config->minSpeedAfterFinish, config->maxSpeedAfterFinish, wheelBaseInCm, config->corneringSpeedCoefficient, downward_accelerationCm, this->angleHeadingTarget);
        }
        else
        {
            this->speed = CalculateCarSpeed(config->minSpeed, config->maxSpeed, wheelBaseInCm, config->corneringSpeedCoefficient, downward_accelerationCm, this->angleHeadingTarget);
        }
    }

    this->steeringAngleServo = calculateServoValue(this->angleHeadingTarget, this->lookAheadDistanceInCm);                 
}


double PurePursuit::adjustSpeed(double trackCurvatureRadius, double currentSpeed) 
{
    
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
                                cv::Point2f carInFramePositionBirdsEye, double pixelSizeInCm, double speed) 
{

    // std::cout << "trackCurvatureRadius before K: " << trackCurvatureRadius << "\n";
    
    static double k = computeK(trackCurvatureRadius);  // Initialize once

    //std::cout << "trackCurvatureRadius: " << trackCurvatureRadius << "  | k: " << k << "\n";
    /* Warning: We enter a corner with e.g. 270cm/s and we want minLookAhead to be 40cm
     *          k must be 0.148 so we have it configured as 14.8 and we divide by 100 (DO NOT CHANGE)
     */
    //k = (0.8 * k + 0.2 * computeK(trackCurvatureRadius));  // Update with smoothing
    k = computeK(trackCurvatureRadius);
    k = k / 100;
    double minimumDistance = pixelSizeInCm * shortestDistanceToCurve(allMidPoints, carInFramePositionBirdsEye);

    //CHANGE BEFORE OFFICIAL RUN
    //double lookAheadDistance = k + minimumDistance;
    double lookAheadDistance = k * speed + minimumDistance;
    lookAheadDistance = std::max(config->minLookAheadInCm, std::min(config->maxLookAheadInCm, lookAheadDistance));

    return lookAheadDistance;
}

cv::Point2f PurePursuit::findHighestIntersection(const std::vector<cv::Point2f>& curve, const cv::Point2f& circleCenter, double circleRadius) 
{
    
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
    double k = config->k_min + ratio * (config->k_max - config->k_min);
    return k;
}

// Function to calculate servo value
double PurePursuit::calculateServoValue(double angleRadians, double lookaheadDistance) 
{

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

    if (curve.empty()) {
        std::cerr << "[ERROR] PurePursuit::shortestDistanceToCurve called with empty curve!" << std::endl;
        return std::numeric_limits<float>::max();
    }

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
                                                       const cv::Point2f &carPos) {
                                                        
    if (midLine.size() < 3) {
        return 1e9; // Treat as nearly straight
    }

    int iClosest = findClosestIndex(midLine, carPos);
    double minRadius = std::numeric_limits<double>::max();

    // Search forward until we exceed lookAheadDistance
    for (int i = iClosest; i < midLine.size() - 3; i++) 
    {
        double dist = cv::norm(midLine[i] - carPos);

        const cv::Point2f &A = midLine[i];
        const cv::Point2f &B = midLine[i + 1];
        const cv::Point2f &C = midLine[i + 2];

        double R = computeCurvatureRadius(A, B, C);

        if(R > falsePositiveCurvatureRadius)
        {
            minRadius = std::min(minRadius, R);
        }
    }

    return minRadius;
}
double PurePursuit::carTurnMaxSpeed(double _turn_radius, double _friction_coefficient, double _downward_acceleration) {
	double _max_speed;
	_max_speed = sqrtf(fabs(_friction_coefficient * _turn_radius * _downward_acceleration));
	return _max_speed;
}

double PurePursuit::carBoostExitCornerSpeed(double new_car_speed, double _turn_angle)
{
    const double threshold = 0.35;
    const double boostSpeedValue = 0;

    if (fabs(_turn_angle) < threshold) 
    {
        new_car_speed = new_car_speed + boostSpeedValue;
    }
    return new_car_speed;
}

double PurePursuit::RearWheelTurnRadius(double wheelBase, double turnAngle) {
	double angle;
	//float temp_sin = sinf(turnAngle);
	if (doubleCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	double temp_sin = tanf(turnAngle);
	if (doubleCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	//angle = (wheelBase / tanf(turnAngle));
	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}
double PurePursuit::CalculateCarSpeed(double _min_speed, double _max_speed, double _wheel_base, double _friction_coefficient, double _downward_acceleration, double _turn_angle) {
	double new_car_speed, turn_radius, booster_car_speed;
	turn_radius = RearWheelTurnRadius(_wheel_base, _turn_angle);
	if (doubleCmp(turn_radius, 0.0f) < 0) {
		new_car_speed = _max_speed;
	}
	else{
		new_car_speed = carTurnMaxSpeed(turn_radius, _friction_coefficient, _downward_acceleration);
	}
	
    booster_car_speed = carBoostExitCornerSpeed(new_car_speed, _turn_angle);
    new_car_speed = booster_car_speed;

	new_car_speed = MAX(_min_speed, new_car_speed);
	new_car_speed = MIN(_max_speed, new_car_speed);

	return new_car_speed;
}


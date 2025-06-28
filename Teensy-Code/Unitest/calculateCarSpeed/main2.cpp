#include <math.h>
#include "geometry2D.h"


static float calculateCarSpeed(float minSpeed, float maxSpeed, float maxSteeringWheelAngle, float steeringWheelAngle, LineABC laneMiddleLine, float ki, float kd, float kiMinMaxImpact) {
	float newCarSpeed_bySteeringAngle, speedSpan, angleCurrentTrajectoryAndMiddleLane, newCarSpeed_byTrajectoryAngle;
	static float sumSteeringWheelAngle = 0.0f;
	static float prevSteeringWheelAngleError = 0.0f;
	float derivativeSteeringError;
	LineABC currentTrajectory;

	kiMinMaxImpact = fabs(kiMinMaxImpact);

	speedSpan = maxSpeed - minSpeed;
	maxSteeringWheelAngle = fabsf(maxSteeringWheelAngle);
	steeringWheelAngle = fabsf(steeringWheelAngle);
	steeringWheelAngle = MIN(steeringWheelAngle, maxSteeringWheelAngle);

	derivativeSteeringError = fabs(steeringWheelAngle - prevSteeringWheelAngleError);

	currentTrajectory = yAxisABC();
	angleCurrentTrajectoryAndMiddleLane = fabsf(angleBetweenLinesABC(currentTrajectory, laneMiddleLine));

	newCarSpeed_byTrajectoryAngle = minSpeed + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_byTrajectoryAngle = MAX(newCarSpeed_byTrajectoryAngle, minSpeed);
	newCarSpeed_byTrajectoryAngle = MIN(newCarSpeed_byTrajectoryAngle, maxSpeed);

	newCarSpeed_bySteeringAngle = minSpeed + (((maxSteeringWheelAngle - steeringWheelAngle) / maxSteeringWheelAngle) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_bySteeringAngle = MAX(newCarSpeed_bySteeringAngle, minSpeed);
	newCarSpeed_bySteeringAngle = MIN(newCarSpeed_bySteeringAngle, maxSpeed);
	sumSteeringWheelAngle += steeringWheelAngle;

	sumSteeringWheelAngle = MAX(sumSteeringWheelAngle, ((-kiMinMaxImpact) / ki));
	sumSteeringWheelAngle = MIN(sumSteeringWheelAngle, (kiMinMaxImpact / ki));

	prevSteeringWheelAngleError = steeringWheelAngle;

	return (float)MIN(newCarSpeed_bySteeringAngle, newCarSpeed_byTrajectoryAngle);
}


int main() {
	float Ki = 0.0f, Kd = 0.0f;
	float speed;

	speed = calculateCarSpeed(97, 135, 55.0f, 55.0f, yAxisABC(), Ki, Kd, 5.0f);
	speed = calculateCarSpeed(97, 135, 55.0f, 0.0f, yAxisABC(), Ki, Kd, 5.0f);
	speed = calculateCarSpeed(97, 135, 55.0f, 10.0f, yAxisABC(), Ki, Kd, 5.0f);


	return 0;
}


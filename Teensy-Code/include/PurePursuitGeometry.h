/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/




#ifndef __PUREPURSUITGEOMETRY_H__
#define __PUREPURSUITGEOMETRY_H__

#include "geometry2D.h"

typedef struct PurePursuitInfo {
	Point2D carPos;
	Point2D nextWayPoint;
	Point2D turnPoint;
	float distanceToWayPoint;
	float lookAheadDistance;
	float TrajectoryToWayPointAngle;
	float steeringAngle;
	float carLength;
	float turnRadius;
	float manouvreLength;
}PurePursuitInfo;

static float carTrajectoryAndWayPointAngle(Point2D carPos, Point2D nextWayPoint) {
	float lookAheadDistance, TrajectoryToWayPointAngle;
	Point2D temp;

	lookAheadDistance = euclidianDistance(carPos, nextWayPoint);
	temp = carPos;
	temp.y += lookAheadDistance;
	TrajectoryToWayPointAngle = triangleAngleA(lookAheadDistance, euclidianDistance(nextWayPoint, temp), lookAheadDistance);
	
	if (floatCmp(carPos.x, nextWayPoint.x) < 0) {
		TrajectoryToWayPointAngle = -TrajectoryToWayPointAngle;
	}
	return TrajectoryToWayPointAngle;
}

static float steeringWheelAngle(float TrajectoryToWayPointAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = atanf((2.0f * carLength * sinf(TrajectoryToWayPointAngle)) / nextWayPointDistance);
	return angle;
}

static float turnRadius(float TrajectoryToWayPointAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = (nextWayPointDistance / (2.0f * sinf(TrajectoryToWayPointAngle)));
	return angle;
}

static float purePursuitComputeSteeringWheelAngle(Point2D carPos, LineMQ wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineMQ(carPos, wayPoints);
	
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircleMQ(carPos, lookAheadDistance, wayPoints);
	
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}

	temp = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	return steeringWheelAngle(temp, carLength, lookAheadDistance);
}

static PurePursuitInfo purePursuitComputeMQ(Point2D carPos, LineMQ wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineMQ(carPos, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircleMQ(carPos, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.carLength = carLength;
	info.turnRadius = turnRadius(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;

	

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}

static PurePursuitInfo purePursuitComputeABC(Point2D carPos, LineABC wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineABC(carPos, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.1f);
	}

	intersectionPoints = intersectionLineCircleABC(carPos, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.carLength = carLength;
	info.turnRadius = turnRadius(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}


#endif // !__PUREPURSUITGEOMETRY_H__

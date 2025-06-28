#define DEBUG_UNIT_TEST
#include "PowerTrain.h"

// wheel diameter = 64mm

int main() {
	float kP = 1.0;
	float kI = 0.1;
	float kD = 0.01;
	float wheelDiameter = 0.064; // exemplu: diametru roata in metri

	PowerTrain powerTrain(kP, kI, kD, 2.0, wheelDiameter, 2.2, 7.4, 930);

	// Exemplu de setare a vitezei
	float speed = 50.0; // viteza in metri/secunda
	float leftWheelPercent = 0.8; // procentul rotii stangi
	float rightWheelPercent = 0.7; // procentul rotii drepte

	powerTrain.SetLeftWheelAccelerationRequest(1.0);

	float deltaTime = 0.1; // Perioada de eșantionare în secunde
	int symulationTime_s = 10; // Simulation time in seconds

	powerTrain.leftWheel.Simulate(deltaTime, symulationTime_s);

	return 0;
}


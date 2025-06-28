#ifndef __POWERTRAIN_H__
#define __POWERTRAIN_H__
#include <math.h>
//#include "geometry2D.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
//TO DO:
//-int read_encoder(int wheel_id); functie pentru encoder
//-void WriteToMotor(float speed)
//-Chose a better rolling resistance coefficient!

#ifdef DEBUG_UNIT_TEST
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
class CarModel {
private:
	double mass;
	double friction;
	double radius;

public:
	CarModel(double mass_, double friction_, double wheel_radius) : mass(mass_), friction(friction_), radius(wheel_radius) {}

	double calculateRPM(double force, double time) {
		// Calculăm termenii funcției de transfer
		double K = 30 / (M_PI * radius);
		double a = mass;
		double b = friction;

		// Calculăm răspunsul în timp (time domain) pentru o intrare constantă (force)
		double exponent = -b / a * time;
		double omega = (K * force / b) * (1 - std::exp(exponent));

		return omega;
	}
};
#endif // DEBUG_UNIT_TEST
/*
float CalculateTorque(float mass, float speed_ms, float radius) {
	const float gForce = 9.81; //Acceleration due to gravity (m/s^2)
	const float C_rr = 0.015; //rolling resistance coefficient/coeficientul de frecare
	
	float fRolling = C_rr * mass * gForce;
	float pRolling = fRolling * speed_ms;

	float acceleration;
	float pAcceleration = 0.5 * mass * acceleration * speed_ms;

	float pTotal = pRolling + pAcceleration;

	float angularVelocity = speed_ms / radius;

	float torque = pTotal / angularVelocity;
	return torque;
}
*/
class Engine
{
public:
	Engine(float batteryAmperage, float batteryVoltage, float motorKV) {
		this->torqueRequest = 0.0;
		this->speedRequest_raw = 0.0;

		this->batteryAmperage = batteryAmperage;	// 2.2A
		this->batteryVoltage = batteryVoltage;		// 7.4V
		this->motorKV = motorKV;					// 930kV
		this->maxTorque = (60.0 * this->batteryAmperage) / (this->motorKV * 2.0 * M_PI); // Torque max = 0.0226 Nm
	}

	void SetTorqueRequest(float torque) {
		torque = MAX(torque, -this->maxTorque);
		torque = MIN(torque, this->maxTorque);
		this->torqueRequest = torque;
		this->SetToMotorRawRequest(this->TorqueToRawValue(torque));
	}

	float GetTorqueRequest() {
		return this->torqueRequest;
	}

	float GetSpeedRequest_raw() {
		return this->speedRequest_raw;
	}



protected:
	
	float TorqueToRawValue(float torque) {
		float rawValue;

		rawValue = torque / this->maxTorque;
		rawValue = (rawValue * 90.0) + 90.0;

		return rawValue;
	}

	static float RPMtoTorque(float RPM, float voltage, float current) {
		float angularSpeed = (RPM * 2 * M_PI) / 60.0; //rad/s

		float power = voltage * current; //watts

		float torqueFromPower = power / angularSpeed; //Nm (Newton-metri)

		return torqueFromPower;
	}

	static float TorqueToRPM(float torque, float voltage, float current) {
		// Calculate power in Watts
		float power = voltage * current;

		// Calculate angular speed in rad/s using torque
		float angularSpeed = power / torque; // rad/s

		// Convert angular speed to RPM
		float RPM = (angularSpeed * 60.0) / (2 * M_PI); // RPM

		return RPM;

	}

	void SetToMotorRawRequest(float value) {
		this->speedRequest_raw = value;
		WriteToMotor(value);
	}

	//=================================TO DO================================================
	virtual void WriteToMotor(float speed) {
		#ifdef DEBUG_UNIT_TEST
			printf("Motor set to speed %.2f (0-180) \n", speed);
		#endif // DEBUG_UNIT_TEST
	}
	//=================================TO DO================================================

	float torqueRequest; 

	float motorKV;
	float batteryVoltage;
	float batteryAmperage;
	float speedRequest_raw;

	float maxTorque;
};

class Wheel: protected Engine //read RPM and set torque
{
public:
	Wheel(float kP, float kI, float kD, float carWeight_, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV): Wheel::Engine(batteryAmperage, batteryVoltage, motorKV) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		this->carWeight = carWeight_;
		this->wheelDiameter = wheelDiameter_meters;

		this->accelerationRequest = 0.0;
		this->prevWheelRPM = 0.0;
		this->wheelRPM = 0.0;
		this->timePassedFromLastSample = 0.0;
	}

	Wheel(float carWeight_, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV): Wheel::Engine(batteryAmperage, batteryVoltage, motorKV) {
		this->wheelDiameter = wheelDiameter_meters;
		this->carWeight = carWeight_;

		this->accelerationRequest = 0.0;
		this->prevWheelRPM = 0.0;
		this->wheelRPM = 0.0;
		this->timePassedFromLastSample = 0.0;
	}

	void SetPID(float kP, float kI, float kD) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}
	
	float GetSpeed() {
		return this->RPMToMps(this->wheelRPM);
	}
	


#ifdef DEBUG_UNIT_TEST
	//==========================SIMULATION FUNCTION==============================
	void Simulate(float deltaTime, float cycles) {
		float leftMeasuredSpeed, rightMeasuredSpeed;
		// Parametrii funcției de transfer
		double mass = this->carWeight; // kg
		double friction = 0.5; // N*s/m
		CarModel carModel(mass, friction, this->wheelDiameter / 2.0);
		for (int i = 0; i < (int)(cycles / deltaTime); ++i) {
			//leftMeasuredSpeed = leftEngine.GetMeasuredSpeed();
			//rightMeasuredSpeed = rightEngine.GetMeasuredSpeed();

			//leftEngine.SetMeasuredSpeed(leftMeasuredSpeed + (leftEngine.GetSpeedRequest_raw() - leftMeasuredSpeed) * 0.1, deltaTime);
			//rightEngine.SetMeasuredSpeed(rightMeasuredSpeed + (rightEngine.GetSpeedRequest_raw() - rightMeasuredSpeed) * 0.1, deltaTime);

			this->SetMeasuredRPM(carModel.calculateRPM(this->GetTorqueRequest(), i * deltaTime), deltaTime);

			printf("Cycle %d:\n", i);
			printf("acc: %.2f, Measured: %.2f m/^2\n", this->GetAccelerationRequest(), this->GetAcceleration());

			//leftEngine.SetToMotorSpeedRequest();
			//rightEngine.SetToMotorSpeedRequest();
		}
	}
	//==========================SIMULATION FUNCTION==============================
#endif // DEBUG_UNIT_TEST

	void SetSpeedRequest(float speed_mps) { // speed = mps!
		this->speedRequest = speed_mps;
		this->SetTorqueRequest(this->CalculateTorqueRequest());
	}

	void SetAccelerationRequest(float acceleration_mps2) {
		this->accelerationRequest = acceleration_mps2;
		this->SetTorqueRequest(this->WheelTorqueToEngineTorque(this->AccelerationToTorque(this->accelerationRequest)));
	}

	float GetAccelerationRequest() {
		return this->accelerationRequest;
	}

	float GetAcceleration() {
		float accel = (RPMToMps(this->wheelRPM) - RPMToMps(this->prevWheelRPM)) / this->timePassedFromLastSample;
		return accel;
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) {
		this->prevWheelRPM = this->wheelRPM;
		this->wheelRPM = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->SetTorqueRequest(this->CalculateTorqueRequestFromAcceleration());
	}

private:

	float WheelTorqueToEngineTorque(float wheelTorque) {
		return wheelTorque * (14.0 / 85.0);
	}

	float MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}

	float RPMToMps(float speedRPM) { //RPM --> metri/s
		return (((wheelDiameter / 2.0) * (2 * M_PI)) / 60.0) * speedRPM;
	}

	//carSpeed = calculateCarSpeed(), viteza dorita.
	float CalculateTorqueRequest() { 
		float error = this->speedRequest - this->RPMToMps(this->wheelRPM);
		this->integral += error * this->timePassedFromLastSample;
		float derivative = (error - previous_error) / this->timePassedFromLastSample;
		float output_speed = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * derivative);
		previous_error = error;

		output_speed = fmax(0.0, fmin(output_speed, 180.0));
		return output_speed; 
	}


	float CalculateTorqueRequestFromAcceleration() {
		float output;
		float gg1, gg2, gg3;
		// output = this->GetTorqueRequest() + (this->AccelerationToTorque(this->GetAccelerationRequest() - this->GetAcceleration()));
		gg1 = this->GetAcceleration();
		gg2 = this->GetAccelerationRequest();
		gg3 = this->WheelTorqueToEngineTorque(this->AccelerationToTorque(gg2 - gg1));
		output = this->GetTorqueRequest() + gg3;
		return output;
	}



	float AccelerationToTorque(float acceleration) {
		float required_force, wheel_radius, linear_torque, moment_of_inertia, angular_acceleration, rotational_acceleration_torque, total_torque;

		wheel_radius = this->wheelDiameter / 2.0;
		required_force = this->carWeight * acceleration;
		linear_torque = required_force * wheel_radius;
		moment_of_inertia = 0.5*this->carWeight * (wheel_radius * wheel_radius);
		angular_acceleration = acceleration / wheel_radius;
		rotational_acceleration_torque = moment_of_inertia * angular_acceleration;
		total_torque = linear_torque + rotational_acceleration_torque;

		return total_torque;
	}

	float prevWheelRPM;
	float wheelRPM;
	float wheelDiameter;

	float timePassedFromLastSample;
	float speedRequest;
	float accelerationRequest;
	float previous_error;
	float integral;

	float carWeight;

	float Kp;
	float Ki;
	float Kd;
};

class PowerTrain
{
public:

	PowerTrain(float kP, float kI, float kD, float carWeight, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV)
		:leftWheel(kP, kI, kD, carWeight, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV),
		rightWheel(kP, kI, kD, carWeight, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV) 
	{
		this->carWeight = carWeight;
	}

	PowerTrain(float carWeight, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV)
		:leftWheel(carWeight, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV),
		rightWheel(carWeight, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV)
	{
		this->carWeight = carWeight;
	}

	void SetLeftWheelPID(float kP, float kI, float kD) {
		this->leftWheel.SetPID(kP, kI, kD);
	}
	void SetRightWheelPID(float kP, float kI, float kD) {
		this->rightWheel.SetPID(kP, kI, kD);
	}
	
	float GetRightWheelSpeed() {
		return this->leftWheel.GetSpeed();
	}
	float GetLeftWheelSpeed() {
		return this->rightWheel.GetSpeed();
	}

	void SetRightWheelSpeedRequest(float speed_mps) {
		this->rightWheel.SetSpeedRequest(speed_mps);
	}
	void SetLeftWheelSpeedRequest(float speed_mps) {
		this->leftWheel.SetSpeedRequest(speed_mps);
	}

	float GetRightAcceleration() {
		return this->leftWheel.GetAcceleration();
	}
	float GetLeftWheelAcceleration() {
		return this->rightWheel.GetAcceleration();
	}

	void SetRightWheelAccelerationRequest(float speed_mps2) {
		this->rightWheel.SetAccelerationRequest(speed_mps2);
	}
	void SetLeftWheelAccelerationRequest(float speed_mps2) {
		this->leftWheel.SetAccelerationRequest(speed_mps2);
	}
	Wheel leftWheel;
	Wheel rightWheel;

private:
	
	float carWeight;
	//Wheel leftWheel;
	//Wheel rightWheel;

};



#endif // !__POWERTRAIN_H__

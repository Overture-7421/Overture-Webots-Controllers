#pragma once
#include <string>
#include <frc/system/plant/DCMotor.h>
#include <cmath>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <nlohmann/json.hpp>
#include "NTController/NTController.h"

using namespace webots;

class NTMotor: public NTController {
public:
	struct Config {
		std::string Name;
		std::string NtSuffix;
		std::string Model;
		double GearRatio;
		int MotorCount;
		bool MechanicallyInverted;
		std::optional<double> WinchRadius;
	};

	NTMotor(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;
	void UpdateRotational(double motorTorque);
	void UpdateLinear(double motorTorque);
private:
	frc::DCMotor motorModel = frc::DCMotor(0_V, 0_Nm, 0_A, 0_A, 0_rad_per_s);
	Motor *motor;
	PositionSensor *posSensor;
	nt::NetworkTableInstance ntInst;

	nt::DoublePublisher encoderSpeedEntry, encoderPositionEntry, currentEntry,
			torqueAppliedEntry;
	nt::DoubleSubscriber voltageEntry;
	nt::BooleanSubscriber invertedEntry;

	double lastPos = 0;

	double initialPos = 0;

	double gear_ratio = 1.0;
	int motor_count = 1;
	bool mechanically_inverted = false;
	double winch_radius = 0;
	double winch_circumference = 0;

	std::function<void(double)> updateTypeFunction;
};

void to_json(nlohmann::json &j, const NTMotor::Config &c);
void from_json(const nlohmann::json &j, NTMotor::Config &c);

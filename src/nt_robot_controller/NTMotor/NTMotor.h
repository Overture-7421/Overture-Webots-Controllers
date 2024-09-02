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
	};

	NTMotor(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;
private:
	frc::DCMotor motorModel = frc::DCMotor(0_V, 0_Nm, 0_A, 0_A, 0_rad_per_s);
	Motor *motor;
	PositionSensor *posSensor;
	nt::NetworkTableInstance ntInst;

	nt::DoublePublisher encoderSpeedEntry, encoderPositionEntry, currentEntry,
			torqueAppliedEntry, targetPositionEntry;
	nt::DoubleSubscriber voltageEntry;
	nt::BooleanSubscriber invertedEntry;

	double lastPos = 0;

	double initialPos = 0;

	double gear_ratio = 1.0;
	int motor_count = 1;
	bool mechanically_inverted = false;
};

void to_json(nlohmann::json &j, const NTMotor::Config &c);
void from_json(const nlohmann::json &j, NTMotor::Config &c);

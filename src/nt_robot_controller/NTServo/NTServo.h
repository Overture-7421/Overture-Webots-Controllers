#pragma once
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <nlohmann/json.hpp>
#include "NTController/NTController.h"

using namespace webots;

class NTServo: public NTController {
public:
	struct Config {
		std::string Name;
		std::string NtSuffix;
		double MaxPosition;
		double MinPosition;
		bool Inverted;
	};

	NTServo(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;
private:
	Motor *servo;
	PositionSensor *posSensor;
	nt::NetworkTableInstance ntInst;

	nt::DoublePublisher positionEntry, targetPositionEntry;
	nt::BooleanSubscriber invertedEntry, targetPositionEntrySubscriber;

	double maxPosition;
	double minPosition;
	bool inverted;
};

void to_json(nlohmann::json &j, const NTServo::Config &c);
void from_json(const nlohmann::json &j, NTServo::Config &c);

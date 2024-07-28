#pragma once

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <nlohmann/json.hpp>
#include "NTController/NTController.h"

using namespace webots;

class LinkedJoint: public NTController {
public:
	struct Config {
		std::string SourceMotorName;
		std::string TargetMotorName;
		bool Inverted;
	};
	LinkedJoint(Robot *robot, const Config &config);

	void Init() override;
	void Update() override;
private:
	Motor *sourceMotor = nullptr;
	Motor *targetMotor = nullptr;
	bool invert = false;
};

void to_json(nlohmann::json &j, const LinkedJoint::Config &c);
void from_json(const nlohmann::json &j, LinkedJoint::Config &c);

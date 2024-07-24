#pragma once
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <nlohmann/json.hpp>

#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>
#include "../NTController.h"

using namespace webots;

class NTIMU: public NTController {
public:
	struct Config {
		std::string Name;
	};

	NTIMU(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;

private:
	InertialUnit *imuSensor = nullptr;
	nt::NetworkTableInstance ntInst;
	nt::DoublePublisher rollPub, pitchPub, yawPub;
};

void to_json(nlohmann::json &j, const NTIMU::Config &c);
void from_json(const nlohmann::json &j, NTIMU::Config &c);

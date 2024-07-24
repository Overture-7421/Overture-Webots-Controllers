//
// Created by ajahueym on 1/14/24.
//
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <nlohmann/json.hpp>
#include "../NTController.h"
using namespace webots;

class NTCANCoder: public NTController {
public:
	struct Config {
		std::string Name;
		bool Inverted;
	};

	NTCANCoder(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;
private:
	PositionSensor* sensor;
	nt::NetworkTableInstance ntInst;
	nt::DoublePublisher encoderSpeedEntry, encoderPositionEntry;
	double lastPos = 0;
	bool inverted = false;
	double timeStep;
};

void to_json(nlohmann::json &j, const NTCANCoder::Config &c);
void from_json(const nlohmann::json &j, NTCANCoder::Config &c);

#include <webots/Robot.hpp>
#include <networktables/NetworkTableInstance.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include "NTMotor/NTMotor.h"
#include "NTWorldTelemetry/NTWorldTelemetry.h"

using namespace webots;

int main(int argc, char **argv) {
	Robot *robot = new Robot();

	auto ntInst = nt::NetworkTableInstance::GetDefault();

	ntInst.SetServer("localhost");
	std::stringstream ntIdentity;
	ntIdentity << "nt_webots_controller";
	ntInst.StartClient4(ntIdentity.str());

	int timeStep = (int) robot->getBasicTimeStep();

	NTWorldTelemetry worldTelemetry;
	std::vector < NTMotor > motors;
	nlohmann::json j;

	for (int i = 1; i < argc; i++) {
		try {
			j = nlohmann::json::parse(argv[i]);
			if (j.at("type") == "motor") {
				NTMotor::Config conf = j.at("value").template get<
						NTMotor::Config>();
				motors.emplace_back(robot, conf);
			}
		} catch (const std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

	}

	bool initialized = false;
	double t = 0.0;

	while (robot->step(timeStep) != -1) {
		worldTelemetry.Update(t);
		if (!initialized) {
			for (auto &motor : motors) {
				motor.Init();
			}
			initialized = true;
		} else {
			for (auto &motor : motors) {
				motor.Update();
			}
		}

		t += (double) timeStep / 1000.0;
	}

	delete robot;
	return 0;
}

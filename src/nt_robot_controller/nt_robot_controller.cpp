#include <webots/Robot.hpp>
#include <networktables/NetworkTableInstance.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include "NTController.h"
#include "NTMotor/NTMotor.h"
#include "NTCanCoder/NTCANCoder.h"
#include "NTIMU/NTIMU.h"
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
	std::vector <std::shared_ptr<NTController>> controllers;
	nlohmann::json j;

	for (int i = 1; i < argc; i++) {
		try {
			j = nlohmann::json::parse(argv[i]);
			std::string type = j.at("type");
			if (type == "motor") {
				NTMotor::Config conf = j.at("value").template get<
						NTMotor::Config>();
				controllers.emplace_back(std::make_shared<NTMotor>(robot, conf));
			}else if(type == "cancoder") {
				NTCANCoder::Config conf = j.at("value").template get<
						NTCANCoder::Config>();
				controllers.emplace_back(std::make_shared<NTCANCoder>(robot, conf));
			}else if(type == "imu") {
				NTIMU::Config conf = j.at("value").template get<
						NTIMU::Config>();
				controllers.emplace_back(std::make_shared<NTIMU>(robot, conf));
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
			for (auto &controller : controllers) {
				controller->Init();
			}
			initialized = true;
		} else {
			for (auto &controller : controllers) {
				controller->Update();
			}
		}

		t += (double) timeStep / 1000.0;
	}

	delete robot;
	return 0;
}

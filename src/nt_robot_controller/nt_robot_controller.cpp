#include <webots/Robot.hpp>
#include <networktables/NetworkTableInstance.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include "NTController/NTController.h"
#include "LinkedJoint/LinkedJoint.h"
#include "NTMotor/NTMotor.h"
#include "NTCanCoder/NTCANCoder.h"
#include "NTIMU/NTIMU.h"
#include <NTCamera/NTCamera.h>
#include "NTWorldTelemetry/NTWorldTelemetry.h"

using namespace webots;

int main(int argc, char **argv) {
	Robot *robot = new Robot();

	auto ntInst = nt::NetworkTableInstance::GetDefault();
	ntInst.SetServer("localhost");
	std::stringstream ntIdentity;
	ntIdentity << "nt_webots_controller";
	ntInst.StartClient4(ntIdentity.str());

	NTController::SetTimeStep(robot);

	NTWorldTelemetry worldTelemetry;
	std::vector < std::unique_ptr < NTController >> controllers;
	nlohmann::json j;

	for (int i = 1; i < argc; i++) {
		try {
			j = nlohmann::json::parse(argv[i]);
			std::string type = j.at("type");
			if (type == "motor") {
				NTMotor::Config conf = j.at("value").template get<
						NTMotor::Config>();
				controllers.emplace_back(
						std::make_unique < NTMotor > (robot, conf));
			} else if (type == "cancoder") {
				NTCANCoder::Config conf = j.at("value").template get<
						NTCANCoder::Config>();
				controllers.emplace_back(
						std::make_unique < NTCANCoder > (robot, conf));
			} else if (type == "imu") {
				NTIMU::Config conf =
						j.at("value").template get<NTIMU::Config>();
				controllers.emplace_back(
						std::make_unique < NTIMU > (robot, conf));
			} else if (type == "camera") {
				NTCamera::Config conf = j.at("value").template get<
						NTCamera::Config>();
				controllers.emplace_back(
						std::make_unique < NTCamera > (robot, conf));
			} else if (type == "linked") {
				LinkedJoint::Config conf = j.at("value").template get<
						LinkedJoint::Config>();
				controllers.emplace_back(
						std::make_unique < LinkedJoint > (robot, conf));
			}
		} catch (const std::exception &e) {
			std::cerr << e.what() << std::endl;
		}
	}

	bool initialized = false;
	double t = 0.0;

	double timeToWait = 1; // Seconds
	int nStepsWait = timeToWait / NTController::GetTimeStepSeconds();
	std::cout << "Waiting for simulation..." << std::endl;
	//Let simulation stabilize before enabling controllers...
	for(int i = 0.0; i < nStepsWait; robot->step((int) robot->getBasicTimeStep()), i++) { 
		
	}
	std::cout << "Done" << std::endl;

	while (robot->step((int) robot->getBasicTimeStep()) != -1) {

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

		t += NTController::GetTimeStepSeconds();
		ntInst.Flush();
	}

	delete robot;
	return 0;
}

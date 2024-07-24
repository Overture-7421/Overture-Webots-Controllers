#include "NTIMU.h"

#define RADS_TO_DEGREES (180.0 / M_PI)

NTIMU::NTIMU(Robot *robot, const Config &config) {
	imuSensor = robot->getInertialUnit(config.Name);
	if (imuSensor == nullptr) {
		throw std::runtime_error(
				"IMU with name \"" + config.Name + "\" was not found!!!");
	}

	ntInst = nt::NetworkTableInstance::GetDefault();

	const auto ntable = ntInst.GetTable(robot->getName())->GetSubTable("imu");
	rollPub = ntable->GetDoubleTopic("roll").Publish();
	pitchPub = ntable->GetDoubleTopic("pitch").Publish();
	yawPub = ntable->GetDoubleTopic("yaw").Publish();
	imuSensor->enable(robot->getBasicTimeStep() / 2.0);
}

void NTIMU::Init() {

}

void NTIMU::Update() {
	const auto orientation = imuSensor->getRollPitchYaw();
	rollPub.Set(orientation[0] * RADS_TO_DEGREES);
	pitchPub.Set(orientation[1] * RADS_TO_DEGREES);
	yawPub.Set(orientation[2] * RADS_TO_DEGREES);
}

void to_json(nlohmann::json &j, const NTIMU::Config &c) {
	j = nlohmann::json { { "name", c.Name } };
}

void from_json(const nlohmann::json &j, NTIMU::Config &c) {
	j.at("name").get_to(c.Name);
}

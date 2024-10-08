#include "NTCANCoder.h"
#include <sstream>

NTCANCoder::NTCANCoder(Robot *robot, const Config &config) {
	sensor = robot->getPositionSensor(config.Name);
	inverted = config.Inverted;
	offsetDeg = config.OffsetDeg;

	if (sensor == nullptr) {
		throw std::runtime_error(
				"CanCoder with name \"" + config.Name + "\" was not found!!!");
	}

	ntInst = nt::NetworkTableInstance::GetDefault();

	const auto ntable = ntInst.GetTable(robot->getName())->GetSubTable(
			"cancoders")->GetSubTable(config.Name);
	encoderSpeedEntry = ntable->GetDoubleTopic("cancoder_speed").Publish();
	encoderPositionEntry =
			ntable->GetDoubleTopic("cancoder_position").Publish();

	encoderSpeedEntry.Set(0);
	encoderPositionEntry.Set(0);
	sensor->enable(robot->getBasicTimeStep());
}

void NTCANCoder::Init() {

}

void NTCANCoder::Update() {
	double sensorPosition = sensor->getValue() / (2.0 * M_PI);
	sensorPosition -= offsetDeg / 360.0;

	if (inverted) {
		sensorPosition *= -1;
	}

	encoderPositionEntry.Set(sensorPosition);

	double encoderSpeed = (sensorPosition - lastPos)
			/ NTController::GetTimeStepSeconds();
	lastPos = sensorPosition;

	encoderSpeedEntry.Set(encoderSpeed);
}

void to_json(nlohmann::json &j, const NTCANCoder::Config &c) {
	j = nlohmann::json { { "name", c.Name }, { "inverted", c.Inverted }, {
			"offsetDeg", c.OffsetDeg } };
}

void from_json(const nlohmann::json &j, NTCANCoder::Config &c) {
	j.at("name").get_to(c.Name);
	j.at("inverted").get_to(c.Inverted);
	j.at("offsetDeg").get_to(c.OffsetDeg);
}

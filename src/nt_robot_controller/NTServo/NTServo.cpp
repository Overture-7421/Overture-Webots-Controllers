#include "NTServo.h"
#include <sstream>
#include <iostream>

NTServo::NTServo(Robot *robot, const NTServo::Config &config) {
	servo = robot->getMotor(config.Name);

	if (servo == nullptr) {
		throw std::runtime_error(
				"Servo with name \"" + config.Name + "\" was not found!!!");
	}

	inverted = config.Inverted;

	ntInst = nt::NetworkTableInstance::GetDefault();
	std::stringstream jointName;
	jointName << config.Name;

	if (!config.NtSuffix.empty()) {
		jointName << "_" << config.NtSuffix;
	}

	const auto ntable =
			ntInst.GetTable(robot->getName())->GetSubTable("servos")->GetSubTable(
					jointName.str());
	positionEntry = ntable->GetDoubleTopic("position").Publish();
	targetPositionEntry = ntable->GetDoubleTopic("target_position").Publish();
	invertedEntry = ntable->GetBooleanTopic("inverted").Subscribe(false);

	posSensor = servo->getPositionSensor();
	if (posSensor == nullptr) {
		throw std::runtime_error("Didn't get a position sensor");
	}

	posSensor->enable(robot->getBasicTimeStep());

	positionEntry.Set(0);
	targetPositionEntry.Set(0);
}

void NTServo::Init() {
}

void NTServo::Update() {
	double currentPosition = posSensor->getValue();
	bool isInverted = invertedEntry.Get();

	double targetPosition = targetPositionEntrySubscriber.Get();

	if (isInverted || inverted) {
		targetPosition = maxPosition - (targetPosition - minPosition);
	}

	if (targetPosition < minPosition) {
		targetPosition = minPosition;
	} else if (targetPosition > maxPosition) {
		targetPosition = maxPosition;
	}

	servo->setPosition(targetPosition);

	positionEntry.Set(currentPosition);
}

void to_json(nlohmann::json &j, const NTServo::Config &c) {
	j = nlohmann::json { { "name", c.Name }, { "suffix", c.NtSuffix }, {
			"inverted", c.Inverted } };
}

void from_json(const nlohmann::json &j, NTServo::Config &c) {
	j.at("name").get_to(c.Name);
	j.at("suffix").get_to(c.NtSuffix);
	j.at("inverted").get_to(c.Inverted);
}

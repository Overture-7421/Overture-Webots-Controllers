#include "LinkedJoint.h"
#include <webots/PositionSensor.hpp>

LinkedJoint::LinkedJoint(Robot *robot, const Config &config) {
	sourceMotor = robot->getMotor(config.SourceMotorName);
	targetMotor = robot->getMotor(config.TargetMotorName);

	if (sourceMotor == nullptr || targetMotor == nullptr) {
		throw new std::runtime_error(
				"Source motor " + config.SourceMotorName + " or Target motor "
						+ config.TargetMotorName + " does not exist!!!");
	}

	sourcePositionSensor = sourceMotor->getPositionSensor();

	if(sourcePositionSensor == nullptr) {
		throw new std::runtime_error(
			"Source motor " + config.SourceMotorName + " needs a PositionSensor!!!");
	}
	invert = config.Inverted;
	trackVel = config.TrackVelocity.value_or(false);
	sourcePositionSensor->enable(robot->getBasicTimeStep());
}

void LinkedJoint::Init() {
	lastPos = sourcePositionSensor->getValue();
}

void LinkedJoint::Update() {
	double sourcePos = sourcePositionSensor->getValue();
	if (invert) {
		sourcePos *= -1;
	}

	if(trackVel) {
		double jointSpeed = (sourcePos - lastPos) / NTController::GetTimeStepSeconds();
		lastPos = sourcePos;
		targetMotor->setVelocity(jointSpeed);
	}else{
		targetMotor->setPosition(sourcePos);
	}


}

void to_json(nlohmann::json &j, const LinkedJoint::Config &c) {
	j = nlohmann::json { { "source_name", c.SourceMotorName }, { "target_name",
			c.TargetMotorName }, { "inverted", c.Inverted },  {"track_vel", c.TrackVelocity.value_or(false)} };
}

void from_json(const nlohmann::json &j, LinkedJoint::Config &c) {
	j.at("source_name").get_to(c.SourceMotorName);
	j.at("target_name").get_to(c.TargetMotorName);
	j.at("inverted").get_to(c.Inverted);

	if(j.contains("track_speed")) {
		c.TrackVelocity = j.at("track_vel").get<bool>();
	}
}

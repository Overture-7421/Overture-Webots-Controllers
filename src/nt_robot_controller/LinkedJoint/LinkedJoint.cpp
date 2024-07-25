#include "LinkedJoint.h"

LinkedJoint::LinkedJoint(Robot* robot, const Config& config) {
    sourceMotor = robot->getMotor(config.SourceMotorName);
    targetMotor = robot->getMotor(config.TargetMotorName);

    if(sourceMotor == nullptr || targetMotor == nullptr){
        throw new std::runtime_error("Source motor " + config.SourceMotorName + " or Target motor " + config.TargetMotorName + " does not exist!!!");
    }

    invert = config.Inverted;
}

void LinkedJoint::Init() {

}

void LinkedJoint::Update() {
    double sourceVel = sourceMotor->getVelocity();

    if(invert){
        sourceVel *= -1;
    }

    targetMotor->setVelocity(sourceVel);
}

void to_json(nlohmann::json &j, const LinkedJoint::Config &c) {
	j = nlohmann::json { 
        { "source_name", c.SourceMotorName },
        { "target_name", c.TargetMotorName },
        { "inverted", c.Inverted } 
    };
}

void from_json(const nlohmann::json &j, LinkedJoint::Config &c) {
	j.at("source_name").get_to(c.SourceMotorName);
    j.at("target_name").get_to(c.TargetMotorName);
	j.at("inverted").get_to(c.Inverted);
}

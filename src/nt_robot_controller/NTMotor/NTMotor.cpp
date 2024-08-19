#include "NTMotor.h"
#include <sstream>
#include <iostream>

NTMotor::NTMotor(Robot *robot, const NTMotor::Config &config) {
	std::string modelName = robot->getName();
	motor = robot->getMotor(config.Name);

	if (motor == nullptr) {
		throw std::runtime_error(
				"Motor with name \"" + config.Name + "\" was not found!!!");
	}

	if (config.Model == "Kraken") {
		motorModel = frc::DCMotor::KrakenX60(motor_count);
	} else if (config.Model == "NEO") {
		motorModel = frc::DCMotor::NEO(motor_count);
	} else if (config.Model == "Falcon") {
		motorModel = frc::DCMotor::Falcon500(motor_count);
	} else if (config.Model == "FalconFOC") {
		motorModel = frc::DCMotor::Falcon500FOC(motor_count);
	} else if (config.Model == "Vortex") {
		motorModel = frc::DCMotor::NeoVortex(motor_count);
	} else {
		std::cerr << "NTMotorPlugin used invalid FRC motor model "
				<< config.Model << "!!!!";
		return;
	}

	gear_ratio = config.GearRatio;
	mechanically_inverted = config.MechanicallyInverted;
	motor_count = config.MotorCount;

	motorModel = motorModel.WithReduction(gear_ratio);

	ntInst = nt::NetworkTableInstance::GetDefault();
	std::stringstream jointName;
	jointName << config.Name;

	if (!config.NtSuffix.empty()) {
		jointName << "_" << config.NtSuffix;
	}

	const auto ntable =
			ntInst.GetTable(robot->getName())->GetSubTable("motors")->GetSubTable(
					jointName.str());
	encoderSpeedEntry = ntable->GetDoubleTopic("encoder_speed").Publish();
	encoderPositionEntry = ntable->GetDoubleTopic("encoder_position").Publish();
	currentEntry = ntable->GetDoubleTopic("current").Publish();
	torqueAppliedEntry = ntable->GetDoubleTopic("torque").Publish();
	invertedEntry = ntable->GetBooleanTopic("inverted").Subscribe(false);
	voltageEntry = ntable->GetDoubleTopic("voltage_applied").Subscribe(0.0);

	ntable->GetEntry("voltage_applied").SetDouble(0.0);

	encoderSpeedEntry.Set(0);
	encoderPositionEntry.Set(0);
	currentEntry.Set(0);
	torqueAppliedEntry.Set(0);

	posSensor = motor->getPositionSensor();
	if (posSensor == nullptr) {
		throw std::runtime_error("Didnt get a position sensor");
	}

	posSensor->enable(robot->getBasicTimeStep());
}

void NTMotor::Init() {
	initialPos = posSensor->getValue() / (2.0 * M_PI);
}

void NTMotor::Update() {
	double jointTurns = (posSensor->getValue() - initialPos) / (2.0 * M_PI);
	auto appliedVoltage = units::volt_t(voltageEntry.Get());

	double jointTurnsPerS = (jointTurns - lastPos)
			/ NTController::GetTimeStepSeconds();
	lastPos = jointTurns;

	if (mechanically_inverted) {
		appliedVoltage *= -1;
	}

	auto current = motorModel.Current(
			units::radians_per_second_t(jointTurnsPerS * 2.0 * M_PI * 0.001),
			appliedVoltage);

	auto torqueGenerated = motorModel.Torque(current * 0.001);

	double torqueToApply = torqueGenerated.value();

	motor->setTorque(torqueToApply);

	if (mechanically_inverted) {
		jointTurns *= -1;
		jointTurnsPerS *= -1;
		current *= -1;
	}

	double rotations = jointTurns * gear_ratio;
	encoderPositionEntry.Set(rotations);

	double encoderSpeed = jointTurnsPerS * gear_ratio;
	encoderSpeedEntry.Set(encoderSpeed);

	torqueAppliedEntry.Set(torqueToApply);
	currentEntry.Set(current.value());
}

void to_json(nlohmann::json &j, const NTMotor::Config &c) {
	j =
			nlohmann::json { { "name", c.Name }, { "suffix", c.NtSuffix }, {
					"model", c.Model }, { "gear_ratio", c.GearRatio }, {
					"count", c.MotorCount }, { "mechanically_inverted",
					c.MechanicallyInverted } };
}

void from_json(const nlohmann::json &j, NTMotor::Config &c) {
	j.at("name").get_to(c.Name);
	j.at("suffix").get_to(c.NtSuffix);
	j.at("model").get_to(c.Model);
	j.at("gear_ratio").get_to(c.GearRatio);
	j.at("count").get_to(c.MotorCount);
	j.at("mechanically_inverted").get_to(c.MechanicallyInverted);
}

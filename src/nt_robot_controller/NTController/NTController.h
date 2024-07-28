#pragma once

#include <webots/Robot.hpp>
class NTController {
public:
	virtual void Init() = 0;
	virtual void Update() = 0;
	virtual ~NTController() {
	}

	static void SetTimeStep(webots::Robot *robot) {
		NTController::TimeStepSeconds = robot->getBasicTimeStep() / 1000.0;
	}

	static double GetTimeStepSeconds() {
		return NTController::TimeStepSeconds;
	}

private:
	static double TimeStepSeconds;
};

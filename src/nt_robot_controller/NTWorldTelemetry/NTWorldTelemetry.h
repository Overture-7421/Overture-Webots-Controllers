#pragma once
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <iostream>
#include <webots/Robot.hpp>
using namespace webots;

class NTWorldTelemetry {
public:
    NTWorldTelemetry();

    void Update(double simulationTime);

private:
    nt::NetworkTableInstance ntInst;
    nt::DoublePublisher simTimeEntry;
};
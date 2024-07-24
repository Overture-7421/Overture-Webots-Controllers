#include "NTWorldTelemetry.h"

NTWorldTelemetry::NTWorldTelemetry() {
    ntInst = nt::NetworkTableInstance::GetDefault();
    const auto ntable = ntInst.GetTable("nt_simworld");
    simTimeEntry = ntable->GetDoubleTopic("sim_time").Publish();
}

void NTWorldTelemetry::Update(double simulationTime) {
    simTimeEntry.Set(simulationTime);
}
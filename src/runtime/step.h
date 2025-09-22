#ifndef HINACLOTH_STEP_H
#define HINACLOTH_STEP_H
#include "api/status.h"
#include "api/telemetry.h"

namespace sim {
    struct Model; struct Data; struct SolveOverrides; // SolveOverrides is defined for API in adapter header
    // API-facing runtime step wrapper
    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out);
}
#endif //HINACLOTH_STEP_H

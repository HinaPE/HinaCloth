#ifndef HINACLOTH_STEP_ENG_H
#define HINACLOTH_STEP_ENG_H
#include "core/common/types.h"

namespace sim { struct Model; struct Data; }

namespace sim { namespace eng {
    struct SolveOverrides;
    struct TelemetryFrame;
    enum class Status;
    Status runtime_step(const ::sim::Model& m, ::sim::Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out);
}}

#endif // HINACLOTH_STEP_ENG_H


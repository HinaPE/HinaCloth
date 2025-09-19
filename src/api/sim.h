#ifndef HINACLOTH_SIM_H
#define HINACLOTH_SIM_H
#include "status.h"
#include "build.h"
#include "commands.h"
#include "telemetry.h"
#include "capability.h"

namespace sim {
    struct Solver;
    Result<Solver*> create(const BuildDesc& desc);
    void destroy(Solver* s);
    Status step(Solver* s, float dt);
    Status push_command(Solver* s, const Command& c);
    Status flush_commands(Solver* s, ApplyPhase p);

    struct Chosen {
        DataLayout layout;
        Backend backend;
        int threads;
    };

    Result<Chosen> query_chosen(Solver* s);
    Status telemetry_query_frame(Solver* s, TelemetryFrame* out);
}

#endif //HINACLOTH_SIM_H

#ifndef HINACLOTH_SIM_H
#define HINACLOTH_SIM_H
#include "status.h"
#include "build.h"
#include "commands.h"
#include "telemetry.h"
#include "capability.h"
#include "chosen.h"
#include <cstddef>

namespace sim {
    struct Solver;
    [[nodiscard]] Result<Solver*> create(const BuildDesc& desc);
    void destroy(Solver* s);
    Status step(Solver* s, float dt);
    Status push_command(Solver* s, const Command& c);
    Status flush_commands(Solver* s, ApplyPhase p);

    [[nodiscard]] Result<Chosen> query_chosen(Solver* s);
    [[nodiscard]] Status telemetry_query_frame(Solver* s, TelemetryFrame* out);

    // Copy current positions into dst (interleaved float3: x,y,z).
    // If maxCount==0, copies all nodes. Returns Ok on success.
    Status copy_positions(Solver* s, float* dst, size_t maxCount, size_t* outCount);
}

#endif //HINACLOTH_SIM_H

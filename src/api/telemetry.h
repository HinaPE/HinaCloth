#ifndef HINACLOTH_TELEMETRY_H
#define HINACLOTH_TELEMETRY_H
#include <cstdint>

namespace sim
{
    struct TelemetryFrame
    {
        double   step_ms;
        double   residual_avg;      // average absolute constraint violation (distance) in last frame
        double   last_rebuild_ms;   // duration of the last structural rebuild applied via flush
        double   avg_rebuild_ms;    // exponential moving average or simple average of rebuild durations
        uint64_t commands_applied;
        uint64_t structural_rebuilds;
        int      solve_substeps;    // substeps used in the last frame
        int      solve_iterations;  // iterations per substep used in the last frame
    };
}

#endif //HINACLOTH_TELEMETRY_H
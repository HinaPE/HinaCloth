/*
 * File: telemetry.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_TELEMETRY_H
#define HINACLOTH_TELEMETRY_H
#include <cstdint>

namespace sim
{
    struct TelemetryFrame
    {
        double   step_ms;
        double   residual_avg;
        double   last_rebuild_ms;
        double   avg_rebuild_ms;
        uint64_t commands_applied;
        uint64_t structural_rebuilds;
        int      solve_substeps;
        int      solve_iterations;
    };
}

#endif

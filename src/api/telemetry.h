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
        // Per-phase timings (accumulated across substeps in milliseconds)
        double   integrate_ms;
        double   pack_ms;       // SoA<->AoSoA/AoS pack time
        double   attachment_ms;
        double   distance_ms;
        double   bending_ms;
        double   finalize_ms;
        double   unpack_ms;     // SoA<->AoSoA/AoS unpack time
    };
}

#endif

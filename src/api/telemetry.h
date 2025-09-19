#ifndef HINACLOTH_TELEMETRY_H
#define HINACLOTH_TELEMETRY_H
#include <cstdint>

namespace sim
{
    struct TelemetryFrame
    {
        double step_ms;
        uint64_t commands_applied;
        uint64_t structural_rebuilds;
    };
}

#endif //HINACLOTH_TELEMETRY_H
#ifndef HINACLOTH_SOLVER_INTERNAL_H
#define HINACLOTH_SOLVER_INTERNAL_H
#include "api/sim.h"
#include "adapter/engine_adapter.h"
#include <vector>

namespace sim {
    struct Solver {
        EngineHandle* e;
        TelemetryFrame tf;
        std::vector<Command> before;
        std::vector<Command> after;
        Chosen chosen;
        unsigned long long applied;
        unsigned long long rebuilds;
    };
}
#endif //HINACLOTH_SOLVER_INTERNAL_H

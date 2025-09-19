#include "solver_internal.h"
namespace sim {
    Status step(Solver* s, float dt) {
        if(!s) return Status::InvalidArgs;
        SolveOverrides ovr;
        ovr.substeps_override = 0;
        ovr.iterations_override = 0;
        Status st = engine_step(s->e, dt, &ovr, &s->tf);
        return st;
    }
}

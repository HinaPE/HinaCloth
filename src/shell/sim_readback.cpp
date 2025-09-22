#include "solver_internal.h"
#include "adapter/engine_adapter.h"

namespace sim {
    Status copy_positions(Solver* s, float* dst, size_t maxCount, size_t* outCount) {
        if (!s) return Status::InvalidArgs;
        return engine_copy_positions(s->e, dst, maxCount, outCount);
    }
}

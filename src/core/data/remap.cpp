#include "remap.h"

namespace sim {
    void core_remapplan_destroy(RemapPlan* p) noexcept {
        delete p;
    }
}

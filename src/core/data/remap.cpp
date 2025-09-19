#include "remap.h"

namespace sim {
    void core_remapplan_destroy(RemapPlan* p) {
        delete p;
    }
}

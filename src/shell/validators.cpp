#include "api/build.h"
#include <cstdint>

namespace sim {
    static bool check_state(const StateInit& s) {
        for (size_t i = 0; i < s.field_count; i++) {
            auto& f = s.fields[i];
            if (!f.name) return false;
            if (!f.data) return false;
            if (f.count == 0) return false;
            if (f.components == 0) return false;
            if (f.stride_bytes == 0) return false;
        }
        return true;
    }

    static bool check_topology(const TopologyIn& t) {
        if (t.node_count == 0) return false;
        if (t.relation_count > 0 && !t.relations) return false;
        return true;
    }

    static bool check_policy(const Policy& p) {
        if (p.solve.substeps < 0) return false;
        if (p.solve.iterations < 0) return false;
        return true;
    }

    bool shell_validate(const BuildDesc& d) {
        if (!check_state(d.state)) return false;
        if (!check_topology(d.topo)) return false;
        if (!check_policy(d.policy)) return false;
        return true;
    }
}

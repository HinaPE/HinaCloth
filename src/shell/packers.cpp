#include "api/build.h"

namespace sim {
    static void pack_state(StateInit& s) {
    }

    static void pack_topology(TopologyIn& t) {
    }

    static void pack_parameters(Parameters& p) {
    }

    void shell_pack(BuildDesc& d) {
        pack_state(d.state);
        pack_topology(d.topo);
        pack_parameters(d.params);
    }
}

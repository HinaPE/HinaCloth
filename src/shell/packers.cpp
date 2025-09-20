#include "api/build.h"

namespace sim {
    static void pack_state([[maybe_unused]] StateInit& s) {
    }

    static void pack_topology([[maybe_unused]] TopologyIn& t) {
    }

    static void pack_parameters([[maybe_unused]] Parameters& p) {
    }

    void shell_pack(BuildDesc& d) {
        pack_state(d.state);
        pack_topology(d.topo);
        pack_parameters(d.params);
    }
}

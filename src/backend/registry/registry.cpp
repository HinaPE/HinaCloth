#include "registry.h"

namespace sim {
    bool backends_choose(const Model& m, const PolicyExec& exec, Chosen& out) {
        (void) m;
        out.layout  = exec.layout == DataLayout::Auto ? DataLayout::SoA : exec.layout;
        out.backend = exec.backend == Backend::Auto ? Backend::Native : exec.backend;
        out.threads = exec.threads < 0 ? 1 : exec.threads;
        return true;
    }
}

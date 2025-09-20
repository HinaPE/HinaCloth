#include "registry.h"
#include "cpu.h"

namespace sim {
    bool backends_choose(const Model& m, const PolicyExec& exec, Chosen& out) {
        (void) m;
        // Decide backend first
        Backend chosen_backend;
        if (exec.backend == Backend::Auto) {
        #if defined(HINACLOTH_HAVE_AVX2)
            if (cpu_has_avx2()) chosen_backend = Backend::AVX2;
            else chosen_backend = Backend::Native;
        #else
            chosen_backend = Backend::Native;
        #endif
        } else {
            chosen_backend = exec.backend;
        }
        out.backend = chosen_backend;

        // Layout policy: prefer Blocked (AoSoA) for AVX2 unless explicitly overridden
        if (exec.layout == DataLayout::Auto) {
            out.layout = (chosen_backend == Backend::AVX2) ? DataLayout::Blocked : DataLayout::SoA;
        } else {
            out.layout = exec.layout;
        }

        out.threads = exec.threads == 0 ? -1 : (exec.threads < 0 ? -1 : exec.threads);
        return true;
    }
}

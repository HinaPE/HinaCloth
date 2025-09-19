#include "api/build.h"

namespace sim {
    static uint64_t hash64(uint64_t x) {
        x ^= x >> 33;
        x *= 0xff51afd7ed558ccdull;
        x ^= x >> 33;
        x *= 0xc4ceb9fe1a85ec53ull;
        x ^= x >> 33;
        return x;
    }

    static uint64_t mix_ptr(const void* p) {
        return hash64((uint64_t) (uintptr_t) p);
    }

    static uint64_t acc;

    void shell_cache_track_begin(const BuildDesc& d) {
        acc = 0;
        acc ^= hash64((uint64_t) d.topo.node_count);
        acc ^= mix_ptr(d.topo.relations);
        acc ^= mix_ptr(d.ops.ops);
        acc ^= hash64((uint64_t) d.policy.exec.threads);
        acc ^= hash64((uint64_t) d.policy.solve.iterations);
    }

    void shell_cache_track_end() {
        (void) acc;
    }
}

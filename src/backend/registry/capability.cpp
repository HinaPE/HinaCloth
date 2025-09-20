#include "api/capability.h"
#include "api/policy_in.h"

namespace sim {
    size_t enumerate_capabilities(Capability* out, size_t cap) {
        Capability list[7];
        list[0]  = Capability{Backend::Native, DataLayout::SoA, "native_soa"};
        list[1]  = Capability{Backend::Native, DataLayout::AoS, "native_aos"};
        list[2]  = Capability{Backend::AVX2, DataLayout::SoA, "avx2_soa"};
        list[3]  = Capability{Backend::TBB, DataLayout::SoA, "tbb_soa"};
        list[4]  = Capability{Backend::GPU, DataLayout::SoA, "gpu_soa"};
        list[5]  = Capability{Backend::Native, DataLayout::Blocked, "native_blocked"};
        list[6]  = Capability{Backend::AVX2, DataLayout::Blocked, "avx2_blocked"};
        size_t n = sizeof(list) / sizeof(list[0]);
        if (!out || cap == 0) return n;
        size_t k = n < cap ? n : cap;
        for (size_t i = 0; i < k; i++) out[i] = list[i];
        return n;
    }
}

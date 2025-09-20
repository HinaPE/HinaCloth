#include "api/capability.h"
#include "api/policy_in.h"

namespace sim {
    size_t enumerate_capabilities(Capability* out, size_t cap) {
        Capability list[6];
        size_t n = 0;
        // Always present: Native + SoA
        list[n++] = Capability{Backend::Native, DataLayout::SoA, "native_soa"};
        // Native + Blocked (AoSoA) supported by runtime if chosen
        list[n++] = Capability{Backend::Native, DataLayout::Blocked, "native_blocked"};
    #if defined(HINACLOTH_HAVE_AVX2)
        list[n++] = Capability{Backend::AVX2, DataLayout::SoA, "avx2_soa"};
        list[n++] = Capability{Backend::AVX2, DataLayout::Blocked, "avx2_blocked"};
    #endif
    #if defined(HINACLOTH_HAVE_TBB)
        list[n++] = Capability{Backend::TBB, DataLayout::SoA, "tbb_soa"};
        list[n++] = Capability{Backend::TBB, DataLayout::Blocked, "tbb_blocked"};
    #endif
        if (!out || cap == 0) return n;
        size_t k = n < cap ? n : cap;
        for (size_t i = 0; i < k; i++) out[i] = list[i];
        return n;
    }
}

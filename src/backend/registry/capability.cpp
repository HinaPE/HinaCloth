#include "capability_eng.h"
#include "core/common/types.h"

namespace sim { namespace eng {
    size_t enumerate_capabilities(eng::Capability* out, size_t cap) {
        eng::Capability list[6];
        size_t n = 0;
        // Always present: Native + SoA
        list[n++] = eng::Capability{eng::Backend::Native, eng::DataLayout::SoA, "native_soa"};
        // Native + Blocked (AoSoA)
        list[n++] = eng::Capability{eng::Backend::Native, eng::DataLayout::Blocked, "native_blocked"};
    #if defined(HINACLOTH_HAVE_AVX2)
        list[n++] = eng::Capability{eng::Backend::AVX2, eng::DataLayout::SoA, "avx2_soa"};
        list[n++] = eng::Capability{eng::Backend::AVX2, eng::DataLayout::Blocked, "avx2_blocked"};
    #endif
    #if defined(HINACLOTH_HAVE_TBB)
        list[n++] = eng::Capability{eng::Backend::TBB, eng::DataLayout::SoA, "tbb_soa"};
        list[n++] = eng::Capability{eng::Backend::TBB, eng::DataLayout::Blocked, "tbb_blocked"};
    #endif
        if (!out || cap == 0) return n;
        size_t k = n < cap ? n : cap;
        for (size_t i = 0; i < k; i++) out[i] = list[i];
        return n;
    }
}}

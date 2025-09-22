#include "api/capability.h"
#include "api/policy_in.h"
#include "backend/registry/capability_eng.h"
#include <vector>

namespace sim {
    size_t enumerate_capabilities(Capability* out, size_t cap) {
        // First, query engine-internal capabilities count
        size_t n = eng::enumerate_capabilities(nullptr, 0);
        if (!out || cap == 0) return n;
        // Fetch into a temporary buffer and map
        std::vector<eng::Capability> tmp(n);
        n = eng::enumerate_capabilities(tmp.data(), tmp.size());
        size_t k = n < cap ? n : cap;
        for (size_t i = 0; i < k; ++i) {
            out[i].backend = static_cast<Backend>(tmp[i].backend);
            out[i].layout  = static_cast<DataLayout>(tmp[i].layout);
            out[i].name    = tmp[i].name;
        }
        return n;
    }
}

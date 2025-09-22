/*
 * File: remap.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_REMAP_H
#define HINACLOTH_REMAP_H
#include <cstdint>
#include <vector>

namespace sim {
    struct RemapPlan {
        std::vector<uint32_t> old_to_new;
    };

    void core_remapplan_destroy(RemapPlan* p) noexcept;
}
#endif

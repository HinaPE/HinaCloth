#include "gateway_world.hpp"
#include "core_base/world_core.hpp"
#include <vector>
#include <cstdint>

namespace rphys {

namespace {
    struct world_slot { world_core* ptr{nullptr}; };
    static std::vector<world_slot> g_worlds; // index = id.value - 1

    bool valid(world_id id) {
        if (id.value == 0) return false;
        std::size_t idx = static_cast<std::size_t>(id.value - 1);
        if (idx >= g_worlds.size()) return false;
        return g_worlds[idx].ptr != nullptr;
    }

    world_core* fetch(world_id id) {
        if (!valid(id)) return nullptr;
        return g_worlds[static_cast<std::size_t>(id.value - 1)].ptr;
    }
}

world_id gw_create_world(const world_desc& desc) {
    (void)desc; // suppress unused parameter until mapping fields implemented
    world_config cfg{}; // map fields if/when extended
    world_core* core = create_world_core(cfg);
    if (!core) return world_id{0};

    // find free slot
    for (std::size_t i = 0; i < g_worlds.size(); ++i) {
        if (g_worlds[i].ptr == nullptr) {
            g_worlds[i].ptr = core;
            return world_id{ static_cast<std::uint32_t>(i + 1) };
        }
    }
    g_worlds.push_back(world_slot{core});
    return world_id{ static_cast<std::uint32_t>(g_worlds.size()) };
}

void gw_destroy_world(world_id id) {
    world_core* core = fetch(id);
    if (!core) return; // silently ignore invalid id for now
    destroy_world_core(core);
    g_worlds[static_cast<std::size_t>(id.value - 1)].ptr = nullptr;
}

void gw_step_world(world_id id, double dt) {
    world_core* core = fetch(id);
    if (!core) return;
    step_world_core(core, dt);
}

std::uint64_t gw_world_frame_count(world_id id) {
    world_core* core = fetch(id);
    return core ? core->frame_count : 0ULL;
}

double gw_world_total_time(world_id id) {
    world_core* core = fetch(id);
    return core ? core->total_time : 0.0;
}

} // namespace rphys

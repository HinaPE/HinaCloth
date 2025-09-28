#include "world_core.hpp"
#include <new>

namespace rphys {

world_core* create_world_core(const world_config& cfg) {
    world_core* w = new (std::nothrow) world_core{};
    if (!w) return nullptr;
    w->config = cfg;
    w->frame_count = 0;
    w->total_time = 0.0;
    return w;
}

void destroy_world_core(world_core* w) noexcept {
    delete w;
}

void step_world_core(world_core* w, double dt) {
    if (!w) return;
    if (dt < 0.0) dt = 0.0; // clamp negative dt
    ++w->frame_count;
    w->total_time += dt;
}

} // namespace rphys

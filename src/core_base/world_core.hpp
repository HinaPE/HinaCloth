#ifndef RPHYS_WORLD_CORE_HPP
#define RPHYS_WORLD_CORE_HPP

#include <cstdint>

namespace rphys {

struct world_config { int placeholder{}; };

// Internal core implementation structure (opaque to public API)
struct world_core {
    std::uint64_t frame_count{0};
    double        total_time{0.0};
    world_config  config{};
};

// Factory / lifecycle / stepping (used by gateway layer)
world_core* create_world_core(const world_config&);
void destroy_world_core(world_core*) noexcept;
void step_world_core(world_core*, double dt);

} // namespace rphys

#endif // RPHYS_WORLD_CORE_HPP

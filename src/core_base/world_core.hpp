#ifndef RPHYS_WORLD_CORE_HPP
#define RPHYS_WORLD_CORE_HPP

namespace rphys {

struct world_config { int placeholder{}; };
struct world_core;

world_core* create_world_core(const world_config&);
void destroy_world_core(world_core*) noexcept;
void step_world_core(world_core*, double dt);

} // namespace rphys

#endif // RPHYS_WORLD_CORE_HPP


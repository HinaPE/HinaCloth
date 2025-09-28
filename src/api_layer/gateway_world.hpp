#ifndef RPHYS_GATEWAY_WORLD_HPP
#define RPHYS_GATEWAY_WORLD_HPP

#include <cstdint>
#include "rphys/forward.h"

namespace rphys {

struct world_core;
struct world_config;

// Internal gateway (not part of public stable API) managing id<->pointer mapping.
world_id gw_create_world(const world_desc& desc);
void     gw_destroy_world(world_id id);
void     gw_step_world(world_id id, double dt);
std::uint64_t gw_world_frame_count(world_id id);
double   gw_world_total_time(world_id id);

} // namespace rphys

#endif // RPHYS_GATEWAY_WORLD_HPP

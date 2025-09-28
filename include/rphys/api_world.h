#ifndef RPHYS_API_WORLD_H
#define RPHYS_API_WORLD_H

#include "forward.h"
#include <cstdint>

namespace rphys {

world_id create_world(const world_desc& desc);
void destroy_world(world_id id);
void step_world(world_id id, double dt);

// Query functions (added for minimal vertical slice diagnostics)
std::uint64_t world_frame_count(world_id id);
double world_total_time(world_id id);

} // namespace rphys

#endif // RPHYS_API_WORLD_H

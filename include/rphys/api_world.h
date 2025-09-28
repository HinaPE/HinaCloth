#ifndef RPHYS_API_WORLD_H
#define RPHYS_API_WORLD_H

#include "forward.h"

namespace rphys {

world_id create_world(const world_desc& desc);
void destroy_world(world_id id);
void step_world(world_id id, double dt);

} // namespace rphys

#endif // RPHYS_API_WORLD_H

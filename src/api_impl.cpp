#include "rphys/api_world.h"
#include "rphys/api_domain.h"
#include "rphys/api_algorithm.h"
#include "rphys/api_scene.h"
#include "rphys/api_params.h"
#include "rphys/api_fields.h"
#include "rphys/api_coupling.h"
#include "rphys/api_events.h"
#include "rphys/api_commands.h"
#include "rphys/api_telemetry.h"
#include "rphys/api_capability.h"
#include "rphys/api_version.h"

#include "api_layer/gateway_world.hpp"

namespace rphys {

world_id create_world(const world_desc& desc) { return gw_create_world(desc); }
void destroy_world(world_id id) { gw_destroy_world(id); }
void step_world(world_id id, double dt) { gw_step_world(id, dt); }
std::uint64_t world_frame_count(world_id id) { return gw_world_frame_count(id); }
double world_total_time(world_id id) { return gw_world_total_time(id); }

} // namespace rphys

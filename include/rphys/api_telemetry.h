#ifndef RPHYS_API_TELEMETRY_H
#define RPHYS_API_TELEMETRY_H

#include "forward.h"
#include <cstdint>

namespace rphys {

struct frame_stats { double frame_ms{0.0}; };
const frame_stats* get_last_frame_stats(world_id);

} // namespace rphys

#endif // RPHYS_API_TELEMETRY_H


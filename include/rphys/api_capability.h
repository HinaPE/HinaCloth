#ifndef RPHYS_API_CAPABILITY_H
#define RPHYS_API_CAPABILITY_H

#include "forward.h"
#include <cstddef>

namespace rphys {

struct capability_record { const char* name; };

std::size_t list_algorithms(world_id, capability_record* buffer, std::size_t capacity);
std::size_t list_schedulers(world_id, capability_record* buffer, std::size_t capacity);
std::size_t list_perf_layers(world_id, capability_record* buffer, std::size_t capacity);

} // namespace rphys

#endif // RPHYS_API_CAPABILITY_H


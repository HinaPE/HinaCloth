#ifndef RPHYS_API_FIELDS_H
#define RPHYS_API_FIELDS_H

#include "forward.h"
#include <cstddef>

namespace rphys {

bool get_field(world_id, domain_id, const char* name, field_view& out);
bool set_field(world_id, domain_id, const char* name, const void* data, std::size_t count, std::size_t stride);

} // namespace rphys

#endif // RPHYS_API_FIELDS_H


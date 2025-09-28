#ifndef RPHYS_API_PARAMS_H
#define RPHYS_API_PARAMS_H

#include "forward.h"
#include <cstddef>

namespace rphys {

bool set_param(world_id, const char* name, double value);
double get_param(world_id, const char* name, double default_value = 0.0);

} // namespace rphys

#endif // RPHYS_API_PARAMS_H


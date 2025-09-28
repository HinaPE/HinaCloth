#ifndef RPHYS_API_COUPLING_H
#define RPHYS_API_COUPLING_H

#include "forward.h"

namespace rphys {

coupling_id register_coupling(world_id, const coupling_desc&);
void remove_coupling(world_id, coupling_id);

} // namespace rphys

#endif // RPHYS_API_COUPLING_H


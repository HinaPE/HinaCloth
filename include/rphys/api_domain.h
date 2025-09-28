#ifndef RPHYS_API_DOMAIN_H
#define RPHYS_API_DOMAIN_H

#include "forward.h"

namespace rphys {

domain_id add_domain(world_id world, const domain_desc& desc);
void remove_domain(world_id world, domain_id domain);

} // namespace rphys

#endif // RPHYS_API_DOMAIN_H


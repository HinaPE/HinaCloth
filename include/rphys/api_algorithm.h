#ifndef RPHYS_API_ALGORITHM_H
#define RPHYS_API_ALGORITHM_H

#include "forward.h"

namespace rphys {

algorithm_id register_algorithm(world_id, domain_id, const algorithm_desc&);
void list_algorithms(domain_id);
void select_algorithm(domain_id, algorithm_id);

} // namespace rphys

#endif // RPHYS_API_ALGORITHM_H


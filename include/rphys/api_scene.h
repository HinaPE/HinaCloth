#ifndef RPHYS_API_SCENE_H
#define RPHYS_API_SCENE_H

#include "forward.h"
#include <vector>

namespace rphys {

struct scene_primitive { int type{0}; };
using scene_primitive_list = std::vector<scene_primitive>;

void build_scene(world_id, domain_id, const scene_primitive_list&);

} // namespace rphys

#endif // RPHYS_API_SCENE_H


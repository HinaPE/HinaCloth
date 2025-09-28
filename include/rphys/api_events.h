#ifndef RPHYS_API_EVENTS_H
#define RPHYS_API_EVENTS_H

#include "forward.h"

namespace rphys {

struct event_desc { const char* name; };
bool schedule_event(world_id, const event_desc&);

} // namespace rphys

#endif // RPHYS_API_EVENTS_H


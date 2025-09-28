#ifndef RPHYS_API_COMMANDS_H
#define RPHYS_API_COMMANDS_H

#include "forward.h"

namespace rphys {

struct command_desc { const char* name; };
bool enqueue_command(world_id, const command_desc&);

} // namespace rphys

#endif // RPHYS_API_COMMANDS_H


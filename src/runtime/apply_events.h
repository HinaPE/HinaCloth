#ifndef HINACLOTH_APPLY_EVENTS_H
#define HINACLOTH_APPLY_EVENTS_H
#include <cstddef>

namespace sim {
    struct Model;
    struct Data;
    struct Command;
    bool runtime_apply_events(Model& m, Data& d, const Command* cmds, size_t count);
}
#endif //HINACLOTH_APPLY_EVENTS_H

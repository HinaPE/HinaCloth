#ifndef HINACLOTH_EVENTS_IN_H
#define HINACLOTH_EVENTS_IN_H
#include <cstddef>

namespace sim
{
    enum class EventKind
    {
        AddNodes,
        RemoveNodes,
        AddRelations,
        RemoveRelations,
        ActivateOperator,
        DeactivateOperator,
        SetParam,
        Custom
    };

    struct EventRecord
    {
        double time;
        EventKind kind;
        const void* payload;
        size_t bytes;
    };

    struct EventsScript
    {
        const EventRecord* records;
        size_t count;
    };
}

#endif //HINACLOTH_EVENTS_IN_H
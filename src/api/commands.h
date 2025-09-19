#ifndef HINACLOTH_COMMANDS_H
#define HINACLOTH_COMMANDS_H
#include <cstddef>

namespace sim
{
    enum class ApplyPhase { BeforeFrame, AfterSolve };

    enum class CommandTag
    {
        SetParam,
        EnableOperator,
        DisableOperator,
        AddNodes,
        RemoveNodes,
        AddRelations,
        RemoveRelations,
        SetFieldRegion,
        Custom
    };

    struct Command
    {
        CommandTag tag;
        const void* data;
        size_t bytes;
    };
}

#endif //HINACLOTH_COMMANDS_H
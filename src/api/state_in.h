#ifndef HINACLOTH_STATE_IN_H
#define HINACLOTH_STATE_IN_H
#include <cstddef>

namespace sim
{
    enum class FieldType
    {
        F32,
        I32,
        U32
    };

    struct FieldView
    {
        const char* name;
        FieldType type;
        const void* data;
        size_t count;
        size_t components;
        size_t stride_bytes;
    };

    struct StateInit
    {
        const FieldView* fields;
        size_t field_count;
    };
}

#endif //HINACLOTH_STATE_IN_H
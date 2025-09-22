/*
 * File: capability.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_CAPABILITY_H
#define HINACLOTH_CAPABILITY_H
#include <cstddef>
#include "policy_in.h"

namespace sim
{
    struct Capability
    {
        Backend backend;
        DataLayout layout;
        const char* name;
    };

    size_t enumerate_capabilities(Capability* out, size_t cap);
}

#endif

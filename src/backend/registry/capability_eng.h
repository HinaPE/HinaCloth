/*
 * File: capability_eng.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_CAPABILITY_ENG_H
#define HINACLOTH_CAPABILITY_ENG_H
#include "core/common/types.h"

namespace sim { namespace eng {
    size_t enumerate_capabilities(eng::Capability* out, size_t cap);
}}

#endif

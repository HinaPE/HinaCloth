#ifndef HINACLOTH_CAPABILITY_ENG_H
#define HINACLOTH_CAPABILITY_ENG_H
#include "core/common/types.h"

namespace sim { namespace eng {
    // Returns number of capabilities. If out!=nullptr, fills up to cap entries.
    size_t enumerate_capabilities(eng::Capability* out, size_t cap);
}}

#endif // HINACLOTH_CAPABILITY_ENG_H


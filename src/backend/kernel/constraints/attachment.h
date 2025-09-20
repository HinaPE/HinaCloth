#ifndef HINACLOTH_ATTACHMENT_H
#define HINACLOTH_ATTACHMENT_H
#include <cstddef>
#include "backend/storage/soa.h"

namespace sim {
    // Apply attachment targets to predicted positions.
    // If inv_mass[i] == 0, skip (pinned). Weight w[i] in [0,1].
    void kernel_attachment_apply(SoAView3& pos,
                                 const float* w,
                                 const float* tx,
                                 const float* ty,
                                 const float* tz,
                                 const float* inv_mass,
                                 std::size_t n);
}

#endif //HINACLOTH_ATTACHMENT_H


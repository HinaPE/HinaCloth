/*
 * File: attachment.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_ATTACHMENT_H
#define HINACLOTH_ATTACHMENT_H
#include <cstddef>
#include "backend/storage/soa.h"
#include "backend/storage/aosoa.h"
#include "backend/storage/aos.h"

namespace sim {
    void kernel_attachment_apply(SoAView3& pos,
                                 const float* w,
                                 const float* tx,
                                 const float* ty,
                                 const float* tz,
                                 const float* inv_mass,
                                 std::size_t n);

    void kernel_attachment_apply_aosoa(AoSoAView3& pos,
                                       const float* w,
                                       const float* tx,
                                       const float* ty,
                                       const float* tz,
                                       const float* inv_mass,
                                       std::size_t n);

    void kernel_attachment_apply_aos(AoSView3& pos,
                                     const float* w,
                                     const float* tx,
                                     const float* ty,
                                     const float* tz,
                                     const float* inv_mass,
                                     std::size_t n);
}

#endif

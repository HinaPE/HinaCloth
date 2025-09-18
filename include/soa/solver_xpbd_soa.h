#ifndef HINACLOTH_INCLUDE_SOA_SOLVER_XPBD_SOA_H
#define HINACLOTH_INCLUDE_SOA_SOLVER_XPBD_SOA_H

// XPBD SoA solver API

#include "xpbd_params.h"
#include "soa/cloth_data_soa.h"

namespace HinaPE {

void xpbd_step_native_soa(ClothSOA& cloth, float dt, const XPBDParams& params);
void xpbd_step_tbb_soa(ClothSOA& cloth, float dt, const XPBDParams& params);
void xpbd_step_avx2_soa(ClothSOA& cloth, float dt, const XPBDParams& params);

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_SOA_SOLVER_XPBD_SOA_H

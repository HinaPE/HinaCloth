// XPBD SoA solver API
#ifndef HINACLOTH_SOLVER_XPBD_SOA_H
#define HINACLOTH_SOLVER_XPBD_SOA_H

#include "xpbd_params.h"
#include "soa/cloth_data_soa.h"

namespace HinaPE {

void xpbd_step_soa(ClothSOA& cloth, float dt, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_SOLVER_XPBD_SOA_H


// XPBD AoSoA solver API
#ifndef HINACLOTH_SOLVER_XPBD_AOSOA_H
#define HINACLOTH_SOLVER_XPBD_AOSOA_H

#include "xpbd_params.h"
#include "aosoa/cloth_data_aosoa.h"

namespace HinaPE {

void xpbd_step_native_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params);
void xpbd_step_tbb_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params);
void xpbd_step_avx2_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_SOLVER_XPBD_AOSOA_H

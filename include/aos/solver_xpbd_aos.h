// XPBD AoS solver API
#ifndef HINACLOTH_SOLVER_XPBD_AOS_H
#define HINACLOTH_SOLVER_XPBD_AOS_H

#include "xpbd_params.h"
#include "aos/cloth_data_aos.h"

namespace HinaPE {

void xpbd_step_native_aos(ClothAOS& cloth, float dt, const XPBDParams& params);
void xpbd_step_tbb_aos(ClothAOS& cloth, float dt, const XPBDParams& params);
void xpbd_step_avx2_aos(ClothAOS& cloth, float dt, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_SOLVER_XPBD_AOS_H

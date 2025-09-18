// XPBD aligned SoA solver API
#ifndef HINACLOTH_SOLVER_XPBD_ALIGNED_H
#define HINACLOTH_SOLVER_XPBD_ALIGNED_H

#include "xpbd_params.h"
#include "aligned/cloth_data_aligned.h"

namespace HinaPE {

void xpbd_step_native_aligned(ClothAligned& cloth, float dt, const XPBDParams& params);
void xpbd_step_tbb_aligned(ClothAligned& cloth, float dt, const XPBDParams& params);
void xpbd_step_avx2_aligned(ClothAligned& cloth, float dt, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_SOLVER_XPBD_ALIGNED_H

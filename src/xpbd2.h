#ifndef HINACLOTH_XPBD2_H
#define HINACLOTH_XPBD2_H

#include "xpbd.h"
#include "cloth_data_2.h"

namespace HinaPE {

// XPBD step for the simple SOA ClothData2 container
void xpbd_step_native2(ClothData2& cloth, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_XPBD2_H


#ifndef HINACLOTH_INCLUDE_SOA_CLOTH_DATA_SOA_H
#define HINACLOTH_INCLUDE_SOA_CLOTH_DATA_SOA_H

// SoA cloth data (Structure of Arrays)

#include "cloth_types.h"

#include <vector>

namespace HinaPE {

struct ClothSOA {
    int nx{0};
    int ny{0};
    std::vector<float> x, y, z;
    std::vector<float> px, py, pz;
    std::vector<float> vx, vy, vz;
    std::vector<float> inv_mass;
    std::vector<float> corr_x, corr_y, corr_z;
    std::vector<int> ci, cj;
    std::vector<float> rest_length, compliance, lambda;
    std::vector<ConstraintType> type;
    std::vector<float> last_c, last_dlambda, last_nx, last_ny, last_nz;
    float last_dt{0.0f};
    int last_iterations{0};
};

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_SOA_CLOTH_DATA_SOA_H

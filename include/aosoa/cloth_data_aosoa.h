#ifndef HINACLOTH_INCLUDE_AOSOA_CLOTH_DATA_AOSOA_H
#define HINACLOTH_INCLUDE_AOSOA_CLOTH_DATA_AOSOA_H

// AoSoA cloth data (Array of Structures of Arrays)

#include "cloth_types.h"

#include <vector>

namespace HinaPE {

constexpr int AOSOA_BLOCK = 16; // tune for SIMD/cache

struct ParticleBlock {
    float x[AOSOA_BLOCK]{};
    float y[AOSOA_BLOCK]{};
    float z[AOSOA_BLOCK]{};
    float px[AOSOA_BLOCK]{};
    float py[AOSOA_BLOCK]{};
    float pz[AOSOA_BLOCK]{};
    float vx[AOSOA_BLOCK]{};
    float vy[AOSOA_BLOCK]{};
    float vz[AOSOA_BLOCK]{};
    float inv_mass[AOSOA_BLOCK]{};
    float corr_x[AOSOA_BLOCK]{};
    float corr_y[AOSOA_BLOCK]{};
    float corr_z[AOSOA_BLOCK]{};
};

struct ConstraintBlock {
    int i[AOSOA_BLOCK]{};
    int j[AOSOA_BLOCK]{};
    float rest_length[AOSOA_BLOCK]{};
    float compliance[AOSOA_BLOCK]{};
    float lambda[AOSOA_BLOCK]{};
    ConstraintType type[AOSOA_BLOCK]{};
    float last_c[AOSOA_BLOCK]{};
    float last_dlambda[AOSOA_BLOCK]{};
    float last_nx[AOSOA_BLOCK]{};
    float last_ny[AOSOA_BLOCK]{};
    float last_nz[AOSOA_BLOCK]{};
};

struct ClothAoSoA {
    int nx{0};
    int ny{0};
    int count{0};
    std::vector<ParticleBlock> pblocks;
    int cons_count{0};
    std::vector<ConstraintBlock> cblocks;
    float last_dt{0.0f};
    int last_iterations{0};
};

inline void index_to_block_lane(int idx, int& block, int& lane) noexcept {
    block = idx / AOSOA_BLOCK;
    lane = idx % AOSOA_BLOCK;
}

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_AOSOA_CLOTH_DATA_AOSOA_H

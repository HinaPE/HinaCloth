// AoSoA cloth data (Array of Structures of Arrays)
#ifndef HINACLOTH_CLOTH_DATA_AOSOA_H
#define HINACLOTH_CLOTH_DATA_AOSOA_H

#include <vector>
#include <cmath>
#include <cstddef>
#include "cloth_types.h"

namespace HinaPE {

constexpr int AOSOA_BLOCK = 16; // tune for SIMD/cache

struct ParticleBlock {
    float x[AOSOA_BLOCK], y[AOSOA_BLOCK], z[AOSOA_BLOCK];
    float px[AOSOA_BLOCK], py[AOSOA_BLOCK], pz[AOSOA_BLOCK];
    float vx[AOSOA_BLOCK], vy[AOSOA_BLOCK], vz[AOSOA_BLOCK];
    float inv_mass[AOSOA_BLOCK];
    float corr_x[AOSOA_BLOCK], corr_y[AOSOA_BLOCK], corr_z[AOSOA_BLOCK];
};

struct ConstraintBlock {
    int i[AOSOA_BLOCK], j[AOSOA_BLOCK];
    float rest_length[AOSOA_BLOCK];
    float compliance[AOSOA_BLOCK];
    float lambda[AOSOA_BLOCK];
    ConstraintType type[AOSOA_BLOCK];
    float last_c[AOSOA_BLOCK], last_dlambda[AOSOA_BLOCK];
    float last_nx[AOSOA_BLOCK], last_ny[AOSOA_BLOCK], last_nz[AOSOA_BLOCK];
};

struct ClothAoSoA {
    int nx{0}, ny{0};
    int count{0};
    std::vector<ParticleBlock> pblocks;

    int cons_count{0};
    std::vector<ConstraintBlock> cblocks;

    float last_dt{0.0f};
    int last_iterations{0};
};

inline void index_to_block_lane(int idx, int& b, int& lane) {
    b = idx / AOSOA_BLOCK; lane = idx % AOSOA_BLOCK;
}

inline void build_cloth_grid_aosoa(ClothAoSoA& cloth,
                                   int nx, int ny,
                                   float width, float height,
                                   float start_y,
                                   bool pin_top_corners,
                                   float comp_struct = 1e-6f,
                                   float comp_shear  = 1e-5f,
                                   float comp_bend   = 1e-4f)
{
    cloth = ClothAoSoA{};
    cloth.nx = nx; cloth.ny = ny;
    const float dx = width / (nx - 1);
    const float dy = height / (ny - 1);
    const float startX = -width * 0.5f;

    int n = nx*ny;
    cloth.count = n;
    int nb = (n + AOSOA_BLOCK - 1) / AOSOA_BLOCK;
    cloth.pblocks.resize(nb);
    for (int idx=0; idx<n; ++idx) {
        int b, lane; index_to_block_lane(idx, b, lane);
        int i = idx % nx; int j = idx / nx;
        float X = startX + dx * i;
        float Y = start_y + dy * (ny - 1 - j);
        cloth.pblocks[b].x[lane]=cloth.pblocks[b].px[lane]=X;
        cloth.pblocks[b].y[lane]=cloth.pblocks[b].py[lane]=Y;
        cloth.pblocks[b].z[lane]=cloth.pblocks[b].pz[lane]=0.0f;
        cloth.pblocks[b].vx[lane]=cloth.pblocks[b].vy[lane]=cloth.pblocks[b].vz[lane]=0.0f;
        cloth.pblocks[b].inv_mass[lane]=1.0f;
        cloth.pblocks[b].corr_x[lane]=cloth.pblocks[b].corr_y[lane]=cloth.pblocks[b].corr_z[lane]=0.0f;
    }
    if (pin_top_corners) {
        int b0,l0; index_to_block_lane(0,b0,l0);
        cloth.pblocks[b0].inv_mass[l0]=0.0f;
        int b1,l1; index_to_block_lane(nx-1,b1,l1);
        cloth.pblocks[b1].inv_mass[l1]=0.0f;
    }

    // Constraints
    std::vector<std::tuple<int,int,float,ConstraintType>> cons;
    cons.reserve(n*4);
    auto add_dist = [&](int a, int b, float comp, ConstraintType ct){
        // compute rest length on the fly
        int ba, la; index_to_block_lane(a, ba, la);
        int bb, lb; index_to_block_lane(b, bb, lb);
        float dx0 = cloth.pblocks[ba].x[la] - cloth.pblocks[bb].x[lb];
        float dy0 = cloth.pblocks[ba].y[la] - cloth.pblocks[bb].y[lb];
        float dz0 = cloth.pblocks[ba].z[la] - cloth.pblocks[bb].z[lb];
        float L0 = std::sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0);
        cons.emplace_back(a,b,L0,ct);
    };

    for (int j=0;j<ny;++j) for (int i=0;i<nx;++i) {
        int id = j*nx+i;
        if (i+1<nx) add_dist(id, id+1, comp_struct, ConstraintType::Structural);
        if (j+1<ny) add_dist(id, id+nx, comp_struct, ConstraintType::Structural);
    }
    for (int j=0;j<ny-1;++j) for (int i=0;i<nx-1;++i) {
        int id = j*nx+i;
        add_dist(id, id+nx+1, comp_shear, ConstraintType::Shear);
        add_dist(id+1, id+nx, comp_shear, ConstraintType::Shear);
    }
    for (int j=0;j<ny;++j) for (int i=0;i<nx;++i) {
        int id = j*nx+i;
        if (i+2<nx) add_dist(id, id+2, comp_bend, ConstraintType::Bending);
        if (j+2<ny) add_dist(id, id+2*nx, comp_bend, ConstraintType::Bending);
    }

    cloth.cons_count = static_cast<int>(cons.size());
    int ncb = (cloth.cons_count + AOSOA_BLOCK - 1) / AOSOA_BLOCK;
    cloth.cblocks.resize(ncb);
    for (int k=0; k<cloth.cons_count; ++k) {
        int b, lane; index_to_block_lane(k, b, lane);
        cloth.cblocks[b].i[lane] = std::get<0>(cons[k]);
        cloth.cblocks[b].j[lane] = std::get<1>(cons[k]);
        cloth.cblocks[b].rest_length[lane] = std::get<2>(cons[k]);
        cloth.cblocks[b].compliance[lane] = 0.0f; // base compliance set by solver-scale
        cloth.cblocks[b].lambda[lane] = 0.0f;
        cloth.cblocks[b].type[lane] = std::get<3>(cons[k]);
        cloth.cblocks[b].last_c[lane]=0.0f; cloth.cblocks[b].last_dlambda[lane]=0.0f;
        cloth.cblocks[b].last_nx[lane]=cloth.cblocks[b].last_ny[lane]=cloth.cblocks[b].last_nz[lane]=0.0f;
    }
}

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_AOSOA_H

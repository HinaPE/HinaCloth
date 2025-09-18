// AoSoA cloth data (Array of Structures of Arrays)
#pragma once

#include "cloth_types.h"
#include "common/cloth_grid_utils.h"

#include <cmath>
#include <stdexcept>
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

inline void build_cloth_grid_aosoa(ClothAoSoA& cloth,
                                   int nx, int ny,
                                   float width, float height,
                                   float start_y,
                                   bool pin_top_corners,
                                   float comp_struct = 1e-6f,
                                   float comp_shear  = 1e-5f,
                                   float comp_bend   = 1e-4f)
{
    if (nx < 2 || ny < 2) {
        throw std::invalid_argument{"build_cloth_grid_aosoa requires nx, ny >= 2"};
    }

    cloth = ClothAoSoA{};
    cloth.nx = nx;
    cloth.ny = ny;

    const float dx = width / static_cast<float>(nx - 1);
    const float dy = height / static_cast<float>(ny - 1);
    const float start_x = -width * 0.5f;

    const int particle_count = nx * ny;
    cloth.count = particle_count;
    const int particle_blocks = (particle_count + AOSOA_BLOCK - 1) / AOSOA_BLOCK;
    cloth.pblocks.assign(particle_blocks, ParticleBlock{});

    for (int idx = 0; idx < particle_count; ++idx) {
        int block_index = 0;
        int lane = 0;
        index_to_block_lane(idx, block_index, lane);
        const int i = idx % nx;
        const int j = idx / nx;
        const float x = start_x + dx * static_cast<float>(i);
        const float y = start_y + dy * static_cast<float>(ny - 1 - j);
        auto& pb = cloth.pblocks[block_index];
        pb.x[lane] = pb.px[lane] = x;
        pb.y[lane] = pb.py[lane] = y;
        pb.z[lane] = pb.pz[lane] = 0.0f;
        pb.vx[lane] = pb.vy[lane] = pb.vz[lane] = 0.0f;
        pb.inv_mass[lane] = 1.0f;
        pb.corr_x[lane] = pb.corr_y[lane] = pb.corr_z[lane] = 0.0f;
    }

    if (pin_top_corners) {
        int block_index = 0;
        int lane = 0;
        index_to_block_lane(0, block_index, lane);
        cloth.pblocks[block_index].inv_mass[lane] = 0.0f;
        index_to_block_lane(nx - 1, block_index, lane);
        cloth.pblocks[block_index].inv_mass[lane] = 0.0f;
    }

    const int total_constraints = detail::totalConstraintCount(nx, ny);
    cloth.cons_count = total_constraints;
    const int constraint_blocks = (total_constraints + AOSOA_BLOCK - 1) / AOSOA_BLOCK;
    cloth.cblocks.assign(constraint_blocks, ConstraintBlock{});

    int next_constraint = 0;
    auto emit_constraint = [&](int a, int b, float compliance_value, ConstraintType constraint_type) {
        const int current = next_constraint++;
        int block_index = 0;
        int lane = 0;
        index_to_block_lane(current, block_index, lane);
        auto& block = cloth.cblocks[block_index];
        block.i[lane] = a;
        block.j[lane] = b;
        int ba = 0, la = 0;
        int bb = 0, lb = 0;
        index_to_block_lane(a, ba, la);
        index_to_block_lane(b, bb, lb);
        const auto& pa = cloth.pblocks[ba];
        const auto& pb = cloth.pblocks[bb];
        const float dx0 = pa.x[la] - pb.x[lb];
        const float dy0 = pa.y[la] - pb.y[lb];
        const float dz0 = pa.z[la] - pb.z[lb];
        block.rest_length[lane] = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        block.compliance[lane] = compliance_value;
        block.lambda[lane] = 0.0f;
        block.type[lane] = constraint_type;
        block.last_c[lane] = 0.0f;
        block.last_dlambda[lane] = 0.0f;
        block.last_nx[lane] = block.last_ny[lane] = block.last_nz[lane] = 0.0f;
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) { emit_constraint(id, id + 1, comp_struct, ConstraintType::Structural); }
            if (j + 1 < ny) { emit_constraint(id, id + nx, comp_struct, ConstraintType::Structural); }
        }
    }

    for (int j = 0; j + 1 < ny; ++j) {
        for (int i = 0; i + 1 < nx; ++i) {
            const int id = j * nx + i;
            emit_constraint(id, id + nx + 1, comp_shear, ConstraintType::Shear);
            emit_constraint(id + 1, id + nx, comp_shear, ConstraintType::Shear);
        }
    }

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 2 < nx) { emit_constraint(id, id + 2, comp_bend, ConstraintType::Bending); }
            if (j + 2 < ny) { emit_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending); }
        }
    }
}

} // namespace HinaPE


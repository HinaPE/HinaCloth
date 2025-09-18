// High-performance aligned SoA cloth data for XPBD
#pragma once

#include "cloth_types.h"
#include "common/aligned_allocator.h"
#include "common/cloth_grid_utils.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace HinaPE {

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, 64>>;

struct ClothAligned {
    int nx{0};
    int ny{0};
    AlignedVector<float> x, y, z;
    AlignedVector<float> px, py, pz;
    AlignedVector<float> vx, vy, vz;
    AlignedVector<float> inv_mass;
    AlignedVector<float> corr_x, corr_y, corr_z;
    AlignedVector<int> ci, cj;
    AlignedVector<float> rest_length, compliance, lambda;
    AlignedVector<ConstraintType> type;
    AlignedVector<float> last_c, last_dlambda, last_nx, last_ny, last_nz;
    float last_dt{0.0f};
    int last_iterations{0};
};

inline void build_cloth_grid_aligned(ClothAligned& cloth,
                                     int nx, int ny,
                                     float width, float height,
                                     float start_y,
                                     bool pin_top_corners,
                                     float comp_struct = 1e-6f,
                                     float comp_shear  = 1e-5f,
                                     float comp_bend   = 1e-4f)
{
    if (nx < 2 || ny < 2) {
        throw std::invalid_argument{"build_cloth_grid_aligned requires nx, ny >= 2"};
    }

    cloth = ClothAligned{};
    cloth.nx = nx;
    cloth.ny = ny;

    const float dx = width / static_cast<float>(nx - 1);
    const float dy = height / static_cast<float>(ny - 1);
    const float start_x = -width * 0.5f;

    const int particle_count = nx * ny;
    auto resize = [&](AlignedVector<float>& buffer, float value) {
        buffer.assign(particle_count, value);
    };

    resize(cloth.x, 0.0f);
    resize(cloth.y, 0.0f);
    resize(cloth.z, 0.0f);
    resize(cloth.px, 0.0f);
    resize(cloth.py, 0.0f);
    resize(cloth.pz, 0.0f);
    resize(cloth.vx, 0.0f);
    resize(cloth.vy, 0.0f);
    resize(cloth.vz, 0.0f);
    resize(cloth.corr_x, 0.0f);
    resize(cloth.corr_y, 0.0f);
    resize(cloth.corr_z, 0.0f);
    cloth.inv_mass.assign(particle_count, 1.0f);

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            const float x = start_x + dx * static_cast<float>(i);
            const float y = start_y + dy * static_cast<float>(ny - 1 - j);
            cloth.x[id] = cloth.px[id] = x;
            cloth.y[id] = cloth.py[id] = y;
            cloth.z[id] = cloth.pz[id] = 0.0f;
        }
    }

    if (pin_top_corners) {
        cloth.inv_mass.front() = 0.0f;
        cloth.inv_mass[nx - 1] = 0.0f;
    }

    const int total_constraints = detail::totalConstraintCount(nx, ny);
    cloth.ci.clear();
    cloth.cj.clear();
    cloth.rest_length.clear();
    cloth.compliance.clear();
    cloth.lambda.clear();
    cloth.type.clear();
    cloth.last_c.clear();
    cloth.last_dlambda.clear();
    cloth.last_nx.clear();
    cloth.last_ny.clear();
    cloth.last_nz.clear();

    cloth.ci.reserve(total_constraints);
    cloth.cj.reserve(total_constraints);
    cloth.rest_length.reserve(total_constraints);
    cloth.compliance.reserve(total_constraints);
    cloth.lambda.reserve(total_constraints);
    cloth.type.reserve(total_constraints);
    cloth.last_c.reserve(total_constraints);
    cloth.last_dlambda.reserve(total_constraints);
    cloth.last_nx.reserve(total_constraints);
    cloth.last_ny.reserve(total_constraints);
    cloth.last_nz.reserve(total_constraints);

    auto append_constraint = [&](int a, int b, float compliance_value, ConstraintType constraint_type) {
        const float dx0 = cloth.x[a] - cloth.x[b];
        const float dy0 = cloth.y[a] - cloth.y[b];
        const float dz0 = cloth.z[a] - cloth.z[b];
        const float rest = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        cloth.ci.push_back(a);
        cloth.cj.push_back(b);
        cloth.rest_length.push_back(rest);
        cloth.compliance.push_back(compliance_value);
        cloth.lambda.push_back(0.0f);
        cloth.type.push_back(constraint_type);
        cloth.last_c.push_back(0.0f);
        cloth.last_dlambda.push_back(0.0f);
        cloth.last_nx.push_back(0.0f);
        cloth.last_ny.push_back(0.0f);
        cloth.last_nz.push_back(0.0f);
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) { append_constraint(id, id + 1, comp_struct, ConstraintType::Structural); }
            if (j + 1 < ny) { append_constraint(id, id + nx, comp_struct, ConstraintType::Structural); }
        }
    }

    for (int j = 0; j + 1 < ny; ++j) {
        for (int i = 0; i + 1 < nx; ++i) {
            const int id = j * nx + i;
            append_constraint(id, id + nx + 1, comp_shear, ConstraintType::Shear);
            append_constraint(id + 1, id + nx, comp_shear, ConstraintType::Shear);
        }
    }

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 2 < nx) { append_constraint(id, id + 2, comp_bend, ConstraintType::Bending); }
            if (j + 2 < ny) { append_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending); }
        }
    }
}

} // namespace HinaPE

// SoA cloth data (Structure of Arrays)
#pragma once

#include "cloth_types.h"
#include "common/cloth_grid_utils.h"

#include <cmath>
#include <stdexcept>
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

inline void build_cloth_grid_soa(ClothSOA& cloth,
                                 int nx, int ny,
                                 float width, float height,
                                 float start_y,
                                 bool pin_top_corners,
                                 float comp_struct = 1e-6f,
                                 float comp_shear  = 1e-5f,
                                 float comp_bend   = 1e-4f)
{
    if (nx < 2 || ny < 2) {
        throw std::invalid_argument{"build_cloth_grid_soa requires nx, ny >= 2"};
    }

    cloth = ClothSOA{};
    cloth.nx = nx;
    cloth.ny = ny;

    const float dx = width / static_cast<float>(nx - 1);
    const float dy = height / static_cast<float>(ny - 1);
    const float start_x = -width * 0.5f;

    const int particle_count = nx * ny;
    auto resize_scalar = [&](std::vector<float>& buffer) {
        buffer.assign(particle_count, 0.0f);
    };

    resize_scalar(cloth.x);
    resize_scalar(cloth.y);
    resize_scalar(cloth.z);
    resize_scalar(cloth.px);
    resize_scalar(cloth.py);
    resize_scalar(cloth.pz);
    resize_scalar(cloth.vx);
    resize_scalar(cloth.vy);
    resize_scalar(cloth.vz);
    cloth.inv_mass.assign(particle_count, 1.0f);
    resize_scalar(cloth.corr_x);
    resize_scalar(cloth.corr_y);
    resize_scalar(cloth.corr_z);

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
    cloth.ci.reserve(total_constraints);
    cloth.cj.reserve(total_constraints);
    cloth.rest_length.reserve(total_constraints);
    cloth.compliance.reserve(total_constraints);
    cloth.lambda.clear();
    cloth.lambda.reserve(total_constraints);
    cloth.type.reserve(total_constraints);
    cloth.last_c.clear();
    cloth.last_c.reserve(total_constraints);
    cloth.last_dlambda.clear();
    cloth.last_dlambda.reserve(total_constraints);
    cloth.last_nx.clear();
    cloth.last_nx.reserve(total_constraints);
    cloth.last_ny.clear();
    cloth.last_ny.reserve(total_constraints);
    cloth.last_nz.clear();
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


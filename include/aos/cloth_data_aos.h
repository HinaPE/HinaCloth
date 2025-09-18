// AoS cloth data (Array of Structs)
#pragma once

#include "cloth_types.h"
#include "common/cloth_grid_utils.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace HinaPE {

struct ParticleAOS {
    float x{0.0f}, y{0.0f}, z{0.0f};
    float px{0.0f}, py{0.0f}, pz{0.0f};
    float vx{0.0f}, vy{0.0f}, vz{0.0f};
    float inv_mass{1.0f};
    float corr_x{0.0f}, corr_y{0.0f}, corr_z{0.0f};
};

struct DistanceConstraintAOS {
    int i{0};
    int j{0};
    float rest_length{0.0f};
    float compliance{0.0f};
    float lambda{0.0f};
    ConstraintType type{ConstraintType::Structural};
    float last_c{0.0f};
    float last_dlambda{0.0f};
    float last_nx{0.0f}, last_ny{0.0f}, last_nz{0.0f};
};

struct ClothAOS {
    int nx{0};
    int ny{0};
    std::vector<ParticleAOS> particles;
    std::vector<DistanceConstraintAOS> constraints;
    float last_dt{0.0f};
    int last_iterations{0};
};

inline void build_cloth_grid_aos(ClothAOS& cloth,
                                 int nx, int ny,
                                 float width, float height,
                                 float start_y,
                                 bool pin_top_corners,
                                 float comp_struct = 1e-6f,
                                 float comp_shear  = 1e-5f,
                                 float comp_bend   = 1e-4f)
{
    if (nx < 2 || ny < 2) {
        throw std::invalid_argument{"build_cloth_grid_aos requires nx, ny >= 2"};
    }

    cloth = ClothAOS{};
    cloth.nx = nx;
    cloth.ny = ny;

    const float dx = width / static_cast<float>(nx - 1);
    const float dy = height / static_cast<float>(ny - 1);
    const float start_x = -width * 0.5f;

    const int particle_count = nx * ny;
    cloth.particles.resize(particle_count);

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            const float x = start_x + dx * static_cast<float>(i);
            const float y = start_y + dy * static_cast<float>(ny - 1 - j);
            auto& p = cloth.particles[id];
            p = ParticleAOS{
                .x = x, .y = y, .z = 0.0f,
                .px = x, .py = y, .pz = 0.0f,
                .vx = 0.0f, .vy = 0.0f, .vz = 0.0f,
                .inv_mass = 1.0f,
                .corr_x = 0.0f, .corr_y = 0.0f, .corr_z = 0.0f
            };
        }
    }

    if (pin_top_corners) {
        cloth.particles.front().inv_mass = 0.0f;
        cloth.particles[nx - 1].inv_mass = 0.0f;
    }

    const int total_constraints = detail::totalConstraintCount(nx, ny);
    cloth.constraints.reserve(total_constraints);

    auto add_constraint = [&](int a, int b, float compliance, ConstraintType type) {
        const auto& pa = cloth.particles[a];
        const auto& pb = cloth.particles[b];
        const float dx0 = pa.x - pb.x;
        const float dy0 = pa.y - pb.y;
        const float dz0 = pa.z - pb.z;
        const float rest = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        cloth.constraints.push_back(DistanceConstraintAOS{
            .i = a,
            .j = b,
            .rest_length = rest,
            .compliance = compliance,
            .lambda = 0.0f,
            .type = type
        });
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) { add_constraint(id, id + 1, comp_struct, ConstraintType::Structural); }
            if (j + 1 < ny) { add_constraint(id, id + nx, comp_struct, ConstraintType::Structural); }
        }
    }

    for (int j = 0; j + 1 < ny; ++j) {
        for (int i = 0; i + 1 < nx; ++i) {
            const int id = j * nx + i;
            add_constraint(id, id + nx + 1, comp_shear, ConstraintType::Shear);
            add_constraint(id + 1, id + nx, comp_shear, ConstraintType::Shear);
        }
    }

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 2 < nx) { add_constraint(id, id + 2, comp_bend, ConstraintType::Bending); }
            if (j + 2 < ny) { add_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending); }
        }
    }
}

} // namespace HinaPE

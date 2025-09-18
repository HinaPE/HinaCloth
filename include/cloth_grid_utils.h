#ifndef HINACLOTH_INCLUDE_CLOTH_GRID_UTILS_H
#define HINACLOTH_INCLUDE_CLOTH_GRID_UTILS_H

#include "aligned/cloth_data_aligned.h"
#include "aos/cloth_data_aos.h"
#include "aosoa/cloth_data_aosoa.h"
#include "soa/cloth_data_soa.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace HinaPE::detail {

[[nodiscard]] constexpr bool validGrid(int nx, int ny) noexcept {
    return nx > 0 && ny > 0;
}

[[nodiscard]] constexpr int structuralConstraintCount(int nx, int ny) noexcept {
    if (nx < 2 || ny < 1) {
        return 0;
    }
    const int safe_nx = std::max(nx, 0);
    const int safe_ny = std::max(ny, 0);
    const int horizontal = std::max(0, safe_nx - 1) * safe_ny;
    const int vertical = safe_nx * std::max(0, safe_ny - 1);
    return horizontal + vertical;
}

[[nodiscard]] constexpr int shearConstraintCount(int nx, int ny) noexcept {
    if (nx < 2 || ny < 2) {
        return 0;
    }
    const int safe_nx = std::max(nx, 0);
    const int safe_ny = std::max(ny, 0);
    return 2 * std::max(0, safe_nx - 1) * std::max(0, safe_ny - 1);
}

[[nodiscard]] constexpr int bendingConstraintCount(int nx, int ny) noexcept {
    const int safe_nx = std::max(nx, 0);
    const int safe_ny = std::max(ny, 0);
    const int horizontal = std::max(0, safe_nx - 2) * safe_ny;
    const int vertical = safe_nx * std::max(0, safe_ny - 2);
    return horizontal + vertical;
}

[[nodiscard]] constexpr int totalConstraintCount(int nx, int ny) noexcept {
    return structuralConstraintCount(nx, ny) + shearConstraintCount(nx, ny) + bendingConstraintCount(nx, ny);
}

} // namespace HinaPE::detail

namespace HinaPE {

inline void build_cloth_grid_aos(ClothAOS& cloth,
                                 int nx,
                                 int ny,
                                 float width,
                                 float height,
                                 float start_y,
                                 bool pin_top_corners,
                                 float comp_struct = 1e-6f,
                                 float comp_shear  = 1e-5f,
                                 float comp_bend   = 1e-4f) {
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
            cloth.particles[id] = ParticleAOS{
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

    cloth.constraints.clear();
    cloth.constraints.reserve(detail::totalConstraintCount(nx, ny));

    const auto append_constraint = [&](int a, int b, float compliance, ConstraintType type) {
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
            if (i + 1 < nx) append_constraint(id, id + 1, comp_struct, ConstraintType::Structural);
            if (j + 1 < ny) append_constraint(id, id + nx, comp_struct, ConstraintType::Structural);
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
            if (i + 2 < nx) append_constraint(id, id + 2, comp_bend, ConstraintType::Bending);
            if (j + 2 < ny) append_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending);
        }
    }
}

inline void build_cloth_grid_soa(ClothSOA& cloth,
                                 int nx,
                                 int ny,
                                 float width,
                                 float height,
                                 float start_y,
                                 bool pin_top_corners,
                                 float comp_struct = 1e-6f,
                                 float comp_shear  = 1e-5f,
                                 float comp_bend   = 1e-4f) {
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
    const auto assign_zero = [&](std::vector<float>& buffer) { buffer.assign(particle_count, 0.0f); };

    assign_zero(cloth.x);
    assign_zero(cloth.y);
    assign_zero(cloth.z);
    assign_zero(cloth.px);
    assign_zero(cloth.py);
    assign_zero(cloth.pz);
    assign_zero(cloth.vx);
    assign_zero(cloth.vy);
    assign_zero(cloth.vz);
    assign_zero(cloth.corr_x);
    assign_zero(cloth.corr_y);
    assign_zero(cloth.corr_z);
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
    cloth.ci.clear(); cloth.ci.reserve(total_constraints);
    cloth.cj.clear(); cloth.cj.reserve(total_constraints);
    cloth.rest_length.clear(); cloth.rest_length.reserve(total_constraints);
    cloth.compliance.clear(); cloth.compliance.reserve(total_constraints);
    cloth.lambda.assign(total_constraints, 0.0f);
    cloth.type.clear(); cloth.type.reserve(total_constraints);
    cloth.last_c.assign(total_constraints, 0.0f);
    cloth.last_dlambda.assign(total_constraints, 0.0f);
    cloth.last_nx.assign(total_constraints, 0.0f);
    cloth.last_ny.assign(total_constraints, 0.0f);
    cloth.last_nz.assign(total_constraints, 0.0f);

    const auto append_constraint = [&](int a, int b, float compliance, ConstraintType type) {
        const float dx0 = cloth.x[a] - cloth.x[b];
        const float dy0 = cloth.y[a] - cloth.y[b];
        const float dz0 = cloth.z[a] - cloth.z[b];
        const float rest = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        cloth.ci.push_back(a);
        cloth.cj.push_back(b);
        cloth.rest_length.push_back(rest);
        cloth.compliance.push_back(compliance);
        cloth.type.push_back(type);
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) append_constraint(id, id + 1, comp_struct, ConstraintType::Structural);
            if (j + 1 < ny) append_constraint(id, id + nx, comp_struct, ConstraintType::Structural);
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
            if (i + 2 < nx) append_constraint(id, id + 2, comp_bend, ConstraintType::Bending);
            if (j + 2 < ny) append_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending);
        }
    }
}

inline void build_cloth_grid_aosoa(ClothAoSoA& cloth,
                                   int nx,
                                   int ny,
                                   float width,
                                   float height,
                                   float start_y,
                                   bool pin_top_corners,
                                   float comp_struct = 1e-6f,
                                   float comp_shear  = 1e-5f,
                                   float comp_bend   = 1e-4f) {
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
        int block = 0;
        int lane = 0;
        index_to_block_lane(idx, block, lane);
        const int i = idx % nx;
        const int j = idx / nx;
        const float x = start_x + dx * static_cast<float>(i);
        const float y = start_y + dy * static_cast<float>(ny - 1 - j);
        auto& pb = cloth.pblocks[static_cast<std::size_t>(block)];
        pb.x[lane] = pb.px[lane] = x;
        pb.y[lane] = pb.py[lane] = y;
        pb.z[lane] = pb.pz[lane] = 0.0f;
        pb.vx[lane] = pb.vy[lane] = pb.vz[lane] = 0.0f;
        pb.inv_mass[lane] = 1.0f;
        pb.corr_x[lane] = pb.corr_y[lane] = pb.corr_z[lane] = 0.0f;
    }

    if (pin_top_corners) {
        int block = 0;
        int lane = 0;
        index_to_block_lane(0, block, lane);
        cloth.pblocks[static_cast<std::size_t>(block)].inv_mass[lane] = 0.0f;
        index_to_block_lane(nx - 1, block, lane);
        cloth.pblocks[static_cast<std::size_t>(block)].inv_mass[lane] = 0.0f;
    }

    const int total_constraints = detail::totalConstraintCount(nx, ny);
    cloth.cons_count = total_constraints;
    const int constraint_blocks = (total_constraints + AOSOA_BLOCK - 1) / AOSOA_BLOCK;
    cloth.cblocks.assign(constraint_blocks, ConstraintBlock{});

    int next_constraint = 0;
    const auto emit_constraint = [&](int a, int b, float compliance, ConstraintType type) {
        const int current = next_constraint++;
        int block = 0;
        int lane = 0;
        index_to_block_lane(current, block, lane);
        auto& cb = cloth.cblocks[static_cast<std::size_t>(block)];
        cb.i[lane] = a;
        cb.j[lane] = b;

        int block_a = 0;
        int lane_a = 0;
        int block_b = 0;
        int lane_b = 0;
        index_to_block_lane(a, block_a, lane_a);
        index_to_block_lane(b, block_b, lane_b);
        const auto& pa = cloth.pblocks[static_cast<std::size_t>(block_a)];
        const auto& pb = cloth.pblocks[static_cast<std::size_t>(block_b)];
        const float dx0 = pa.x[lane_a] - pb.x[lane_b];
        const float dy0 = pa.y[lane_a] - pb.y[lane_b];
        const float dz0 = pa.z[lane_a] - pb.z[lane_b];
        cb.rest_length[lane] = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        cb.compliance[lane] = compliance;
        cb.lambda[lane] = 0.0f;
        cb.type[lane] = type;
        cb.last_c[lane] = 0.0f;
        cb.last_dlambda[lane] = 0.0f;
        cb.last_nx[lane] = cb.last_ny[lane] = cb.last_nz[lane] = 0.0f;
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) emit_constraint(id, id + 1, comp_struct, ConstraintType::Structural);
            if (j + 1 < ny) emit_constraint(id, id + nx, comp_struct, ConstraintType::Structural);
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
            if (i + 2 < nx) emit_constraint(id, id + 2, comp_bend, ConstraintType::Bending);
            if (j + 2 < ny) emit_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending);
        }
    }
}

inline void build_cloth_grid_aligned(ClothAligned& cloth,
                                     int nx,
                                     int ny,
                                     float width,
                                     float height,
                                     float start_y,
                                     bool pin_top_corners,
                                     float comp_struct = 1e-6f,
                                     float comp_shear  = 1e-5f,
                                     float comp_bend   = 1e-4f) {
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
    const auto assign_zero = [&](AlignedVector<float>& buffer) { buffer.assign(particle_count, 0.0f); };

    assign_zero(cloth.x);
    assign_zero(cloth.y);
    assign_zero(cloth.z);
    assign_zero(cloth.px);
    assign_zero(cloth.py);
    assign_zero(cloth.pz);
    assign_zero(cloth.vx);
    assign_zero(cloth.vy);
    assign_zero(cloth.vz);
    assign_zero(cloth.corr_x);
    assign_zero(cloth.corr_y);
    assign_zero(cloth.corr_z);
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
    cloth.ci.clear(); cloth.ci.reserve(total_constraints);
    cloth.cj.clear(); cloth.cj.reserve(total_constraints);
    cloth.rest_length.clear(); cloth.rest_length.reserve(total_constraints);
    cloth.compliance.clear(); cloth.compliance.reserve(total_constraints);
    cloth.lambda.clear(); cloth.lambda.reserve(total_constraints);
    cloth.type.clear(); cloth.type.reserve(total_constraints);
    cloth.last_c.clear(); cloth.last_c.reserve(total_constraints);
    cloth.last_dlambda.clear(); cloth.last_dlambda.reserve(total_constraints);
    cloth.last_nx.clear(); cloth.last_nx.reserve(total_constraints);
    cloth.last_ny.clear(); cloth.last_ny.reserve(total_constraints);
    cloth.last_nz.clear(); cloth.last_nz.reserve(total_constraints);

    const auto append_constraint = [&](int a, int b, float compliance, ConstraintType type) {
        const float dx0 = cloth.x[a] - cloth.x[b];
        const float dy0 = cloth.y[a] - cloth.y[b];
        const float dz0 = cloth.z[a] - cloth.z[b];
        const float rest = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
        cloth.ci.push_back(a);
        cloth.cj.push_back(b);
        cloth.rest_length.push_back(rest);
        cloth.compliance.push_back(compliance);
        cloth.lambda.push_back(0.0f);
        cloth.type.push_back(type);
        cloth.last_c.push_back(0.0f);
        cloth.last_dlambda.push_back(0.0f);
        cloth.last_nx.push_back(0.0f);
        cloth.last_ny.push_back(0.0f);
        cloth.last_nz.push_back(0.0f);
    };

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int id = j * nx + i;
            if (i + 1 < nx) append_constraint(id, id + 1, comp_struct, ConstraintType::Structural);
            if (j + 1 < ny) append_constraint(id, id + nx, comp_struct, ConstraintType::Structural);
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
            if (i + 2 < nx) append_constraint(id, id + 2, comp_bend, ConstraintType::Bending);
            if (j + 2 < ny) append_constraint(id, id + 2 * nx, comp_bend, ConstraintType::Bending);
        }
    }
}

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_CLOTH_GRID_UTILS_H

// AoS cloth data (Array of Structs)
#ifndef HINACLOTH_CLOTH_DATA_AOS_H
#define HINACLOTH_CLOTH_DATA_AOS_H

#include <vector>
#include <cmath>
#include "cloth_types.h"

namespace HinaPE {

struct ParticleAOS {
    float x, y, z;
    float px, py, pz;
    float vx, vy, vz;
    float inv_mass;
    float corr_x{0.0f}, corr_y{0.0f}, corr_z{0.0f};
};

struct DistanceConstraintAOS {
    int i, j;
    float rest_length;
    float compliance;
    float lambda;
    ConstraintType type;
    float last_c{0.0f}, last_dlambda{0.0f};
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
    cloth = ClothAOS{};
    cloth.nx = nx; cloth.ny = ny;
    const float dx = width / (nx - 1);
    const float dy = height / (ny - 1);
    const float startX = -width * 0.5f;

    cloth.particles.resize(nx * ny);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            int id = j*nx + i;
            float x = startX + dx * i;
            float y = start_y + dy * (ny - 1 - j);
            float z = 0.0f;
            cloth.particles[id] = ParticleAOS{ x,y,z, x,y,z, 0,0,0, 1.0f };
        }
    }
    if (pin_top_corners) {
        cloth.particles[0].inv_mass = 0.0f;
        cloth.particles[nx-1].inv_mass = 0.0f;
    }

    auto add_dist = [&](int a, int b, float comp, ConstraintType ct){
        float dx0 = cloth.particles[a].x - cloth.particles[b].x;
        float dy0 = cloth.particles[a].y - cloth.particles[b].y;
        float dz0 = cloth.particles[a].z - cloth.particles[b].z;
        float L0 = std::sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0);
        cloth.constraints.push_back(DistanceConstraintAOS{a,b,L0,comp,0.0f,ct});
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
}

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_AOS_H

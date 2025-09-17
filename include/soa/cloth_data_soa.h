// SoA cloth data (Structure of Arrays)
#ifndef HINACLOTH_CLOTH_DATA_SOA_H
#define HINACLOTH_CLOTH_DATA_SOA_H

#include <vector>
#include <cmath>
#include "cloth_types.h"

namespace HinaPE {

struct ClothSOA {
    int nx{0};
    int ny{0};
    // particle arrays
    std::vector<float> x, y, z;
    std::vector<float> px, py, pz;
    std::vector<float> vx, vy, vz;
    std::vector<float> inv_mass;
    // scratch
    std::vector<float> corr_x, corr_y, corr_z;

    // constraints arrays
    std::vector<int> ci, cj;
    std::vector<float> rest_length, compliance, lambda;
    std::vector<ConstraintType> type;
    // debug
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
    cloth = ClothSOA{};
    cloth.nx = nx; cloth.ny = ny;
    const float dx = width / (nx - 1);
    const float dy = height / (ny - 1);
    const float startX = -width * 0.5f;

    int n = nx*ny;
    cloth.x.resize(n); cloth.y.resize(n); cloth.z.resize(n);
    cloth.px.resize(n); cloth.py.resize(n); cloth.pz.resize(n);
    cloth.vx.resize(n); cloth.vy.resize(n); cloth.vz.resize(n);
    cloth.inv_mass.assign(n, 1.0f);
    cloth.corr_x.assign(n, 0.0f); cloth.corr_y.assign(n, 0.0f); cloth.corr_z.assign(n, 0.0f);

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            int id = j*nx + i;
            float X = startX + dx * i;
            float Y = start_y + dy * (ny - 1 - j);
            cloth.x[id]=cloth.px[id]=X; cloth.y[id]=cloth.py[id]=Y; cloth.z[id]=cloth.pz[id]=0.0f;
            cloth.vx[id]=cloth.vy[id]=cloth.vz[id]=0.0f;
        }
    }
    if (pin_top_corners) {
        cloth.inv_mass[0] = 0.0f;
        cloth.inv_mass[nx-1] = 0.0f;
    }

    auto add_dist = [&](int a, int b, float comp, ConstraintType ct){
        float dx0 = cloth.x[a] - cloth.x[b];
        float dy0 = cloth.y[a] - cloth.y[b];
        float dz0 = cloth.z[a] - cloth.z[b];
        float L0 = std::sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0);
        cloth.ci.push_back(a); cloth.cj.push_back(b);
        cloth.rest_length.push_back(L0);
        cloth.compliance.push_back(comp); cloth.lambda.push_back(0.0f);
        cloth.type.push_back(ct);
        cloth.last_c.push_back(0.0f); cloth.last_dlambda.push_back(0.0f);
        cloth.last_nx.push_back(0.0f); cloth.last_ny.push_back(0.0f); cloth.last_nz.push_back(0.0f);
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

#endif // HINACLOTH_CLOTH_DATA_SOA_H

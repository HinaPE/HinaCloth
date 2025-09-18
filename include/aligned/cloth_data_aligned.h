// High-performance aligned SoA cloth data for XPBD
#ifndef HINACLOTH_CLOTH_DATA_ALIGNED_H
#define HINACLOTH_CLOTH_DATA_ALIGNED_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "cloth_types.h"

namespace HinaPE {

// Minimal aligned storage for POD arrays (64-byte aligned)
template <class T>
struct aligned_array {
    T* data{nullptr};
    std::size_t size{0};
    std::size_t capacity{0};

    static constexpr std::size_t k_align = 64;

    aligned_array() = default;
    ~aligned_array() { clear(); }
    aligned_array(const aligned_array&) = delete;
    aligned_array& operator=(const aligned_array&) = delete;

    void reserve(std::size_t n) {
        if (n <= capacity) return;
        std::size_t bytes = n * sizeof(T);
#if defined(_MSC_VER)
        void* p = _aligned_malloc(bytes ? bytes : k_align, k_align);
        if (!p) throw std::bad_alloc{};
#else
        void* p = nullptr;
        if (posix_memalign(&p, k_align, bytes ? bytes : k_align) != 0) throw std::bad_alloc{};
#endif
        if (data) {
            std::memcpy(p, data, size * sizeof(T));
#if defined(_MSC_VER)
            _aligned_free(data);
#else
            std::free(data);
#endif
        }
        data = static_cast<T*>(p);
        capacity = n;
    }

    void resize(std::size_t n) {
        if (n > capacity) reserve(n);
        size = n;
    }

    void clear() {
        if (data) {
#if defined(_MSC_VER)
            _aligned_free(data);
#else
            std::free(data);
#endif
        }
        data = nullptr; size = capacity = 0;
    }

    T& operator[](std::size_t i) { return data[i]; }
    const T& operator[](std::size_t i) const { return data[i]; }
};

struct ClothAligned {
    int nx{0}, ny{0};
    // particle arrays
    aligned_array<float> x, y, z;
    aligned_array<float> px, py, pz;
    aligned_array<float> vx, vy, vz;
    aligned_array<float> inv_mass;
    // constraints (distance)
    aligned_array<int> ci, cj;
    aligned_array<float> rest_length, compliance, lambda;
    aligned_array<ConstraintType> type;
    // debug
    aligned_array<float> last_c, last_dlambda, last_nx, last_ny, last_nz;

    // meta
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
    // reset meta
    cloth.nx = nx; cloth.ny = ny; cloth.last_dt = 0.0f; cloth.last_iterations = 0;
    // clear previous buffers
    cloth.x.clear(); cloth.y.clear(); cloth.z.clear();
    cloth.px.clear(); cloth.py.clear(); cloth.pz.clear();
    cloth.vx.clear(); cloth.vy.clear(); cloth.vz.clear();
    cloth.inv_mass.clear();
    cloth.ci.clear(); cloth.cj.clear();
    cloth.rest_length.clear(); cloth.compliance.clear(); cloth.lambda.clear();
    cloth.type.clear();
    cloth.last_c.clear(); cloth.last_dlambda.clear(); cloth.last_nx.clear(); cloth.last_ny.clear(); cloth.last_nz.clear();
    const float dx = width / (nx - 1);
    const float dy = height / (ny - 1);
    const float startX = -width * 0.5f;
    const int n = nx*ny;
    cloth.x.resize(n); cloth.y.resize(n); cloth.z.resize(n);
    cloth.px.resize(n); cloth.py.resize(n); cloth.pz.resize(n);
    cloth.vx.resize(n); cloth.vy.resize(n); cloth.vz.resize(n);
    cloth.inv_mass.resize(n);

    for (int j=0;j<ny;++j) {
        for (int i=0;i<nx;++i) {
            int id = j*nx + i;
            float X = startX + dx * i;
            float Y = start_y + dy * (ny - 1 - j);
            cloth.x[id]=cloth.px[id]=X; cloth.y[id]=cloth.py[id]=Y; cloth.z[id]=cloth.pz[id]=0.0f;
            cloth.vx[id]=cloth.vy[id]=cloth.vz[id]=0.0f; cloth.inv_mass[id]=1.0f;
        }
    }
    if (pin_top_corners) {
        cloth.inv_mass[0] = 0.0f;
        cloth.inv_mass[nx-1] = 0.0f;
    }

    // Precompute constraints
    std::vector<std::tuple<int,int,float,ConstraintType>> cons;
    cons.reserve(n*4);
    auto add_dist = [&](int a, int b, float comp, ConstraintType ct){
        float dx0 = cloth.x[a] - cloth.x[b];
        float dy0 = cloth.y[a] - cloth.y[b];
        float dz0 = cloth.z[a] - cloth.z[b];
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

    const int m = (int)cons.size();
    cloth.ci.resize(m); cloth.cj.resize(m);
    cloth.rest_length.resize(m); cloth.compliance.resize(m); cloth.lambda.resize(m);
    cloth.type.resize(m); cloth.last_c.resize(m); cloth.last_dlambda.resize(m);
    cloth.last_nx.resize(m); cloth.last_ny.resize(m); cloth.last_nz.resize(m);
    for (int k=0;k<m;++k) {
        cloth.ci[k] = std::get<0>(cons[k]);
        cloth.cj[k] = std::get<1>(cons[k]);
        cloth.rest_length[k] = std::get<2>(cons[k]);
        cloth.compliance[k] = 0.0f;
        cloth.lambda[k] = 0.0f;
        cloth.type[k] = std::get<3>(cons[k]);
        cloth.last_c[k]=cloth.last_dlambda[k]=0.0f; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0.0f;
    }
}

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_ALIGNED_H

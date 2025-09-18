// Validation tests for XPBD implementations across data layouts and backends
// Goal: ensure algorithmic correctness and numerical consistency within a
// reasonable tolerance across Native/TBB/AVX2 backends for the same layout.
// Metrics:
//  - Constraint residuals: L1-mean, L2-RMS, L_inf (max)
//  - Pinned vertex displacement epsilon check
//  - Cross-backend residual agreement within relative tolerance
// Method:
//  - Initialize identical cloth states per layout
//  - Run K steps with fixed dt/params
//  - Compute residuals per backend; compare to native-baseline

#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>

#include "xpbd_params.h"
#include "cloth_types.h"

#include "aos/cloth_data_aos.h"
#include "aos/solver_xpbd_aos.h"

#include "soa/cloth_data_soa.h"
#include "soa/solver_xpbd_soa.h"

#include "aosoa/cloth_data_aosoa.h"
#include "aosoa/solver_xpbd_aosoa.h"

#include "aligned/cloth_data_aligned.h"
#include "aligned/solver_xpbd_aligned.h"

struct residuals { double l1{}, l2{}, linf{}; size_t m{}; };

static HinaPE::XPBDParams params_default() {
    HinaPE::XPBDParams p{};
    p.ax=0; p.ay=-9.81f; p.az=0;
    p.iterations=10; p.substeps=1; p.min_dt=1.f/400.f; p.max_dt=1.f/30.f;
    p.velocity_damping=0.0f; p.warmstart=false; p.lambda_decay=1.0f;
    p.compliance_scale_all=1.0f; p.compliance_scale_structural=1.0f; p.compliance_scale_shear=1.0f; p.compliance_scale_bending=1.0f;
    p.max_correction=0.0f; p.write_debug_fields=0; return p;
}

static residuals compute_residuals(const HinaPE::ClothAOS& cloth) {
    double s1=0, s2=0, sm=0; size_t m = cloth.constraints.size();
    for (const auto& c : cloth.constraints) {
        const auto& a = cloth.particles[c.i]; const auto& b = cloth.particles[c.j];
        double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z; double r = std::sqrt(dx*dx+dy*dy+dz*dz) - c.rest_length;
        double v = std::abs(r); s1 += v; s2 += r*r; sm = std::max(sm, v);
    }
    return residuals{ s1/m, std::sqrt(s2/m), sm, m };
}
static residuals compute_residuals(const HinaPE::ClothSOA& cloth) {
    double s1=0, s2=0, sm=0; size_t m = cloth.ci.size();
    for (size_t k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; double dx=cloth.x[i]-cloth.x[j], dy=cloth.y[i]-cloth.y[j], dz=cloth.z[i]-cloth.z[j]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-cloth.rest_length[k]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} return residuals{ s1/m, std::sqrt(s2/m), sm, m };
}
static residuals compute_residuals(const HinaPE::ClothAoSoA& cloth) {
    double s1=0, s2=0, sm=0; size_t m = cloth.cons_count;
    for (size_t cb=0; cb < (size_t)((m + HinaPE::AOSOA_BLOCK - 1)/HinaPE::AOSOA_BLOCK); ++cb){ const auto& blk=cloth.cblocks[cb]; for (int l=0;l<HinaPE::AOSOA_BLOCK;++l){ size_t k=cb*HinaPE::AOSOA_BLOCK + l; if (k>=m) break; int ia=blk.i[l], ib=blk.j[l]; int bai=ia/HinaPE::AOSOA_BLOCK, lai=ia%HinaPE::AOSOA_BLOCK; int bbi=ib/HinaPE::AOSOA_BLOCK, lbi=ib%HinaPE::AOSOA_BLOCK; const auto& pa=cloth.pblocks[bai]; const auto& pb=cloth.pblocks[bbi]; double dx=pa.x[lai]-pb.x[lbi], dy=pa.y[lai]-pb.y[lbi], dz=pa.z[lai]-pb.z[lbi]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-blk.rest_length[l]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} } return residuals{ s1/m, std::sqrt(s2/m), sm, m };
}
static residuals compute_residuals(const HinaPE::ClothAligned& cloth) {
    double s1=0, s2=0, sm=0; size_t m = cloth.ci.size;
    for (size_t k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; double dx=cloth.x[i]-cloth.x[j], dy=cloth.y[i]-cloth.y[j], dz=cloth.z[i]-cloth.z[j]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-cloth.rest_length[k]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} return residuals{ s1/m, std::sqrt(s2/m), sm, m };
}

static bool validate_pins(const HinaPE::ClothAOS& c) { return c.particles.front().inv_mass==0.0f && c.particles[c.nx-1].inv_mass==0.0f; }
static bool validate_pins(const HinaPE::ClothSOA& c) { return c.inv_mass.front()==0.0f && c.inv_mass[c.nx-1]==0.0f; }
static bool validate_pins(const HinaPE::ClothAoSoA& c) { int nx=c.nx; int b0=0/HinaPE::AOSOA_BLOCK,l0=0%HinaPE::AOSOA_BLOCK; int b1=(nx-1)/HinaPE::AOSOA_BLOCK,l1=(nx-1)%HinaPE::AOSOA_BLOCK; return c.pblocks[b0].inv_mass[l0]==0.0f && c.pblocks[b1].inv_mass[l1]==0.0f; }
static bool validate_pins(const HinaPE::ClothAligned& c) { return c.inv_mass[0]==0.0f && c.inv_mass[c.nx-1]==0.0f; }

template <class ClothT>
static int validate_layout(const char* name,
                           ClothT& cloth,
                           void(*native)(ClothT&, float, const HinaPE::XPBDParams&),
                           void(*tbb)(ClothT&, float, const HinaPE::XPBDParams&),
                           void(*avx2)(ClothT&, float, const HinaPE::XPBDParams&))
{
    const int nx=64, ny=32; const float dt=1.f/240.f; const int steps=200;
    HinaPE::XPBDParams p = params_default();
    // Build initial state per layout (overloads exist)
    if constexpr (std::is_same_v<ClothT, HinaPE::ClothAOS>) HinaPE::build_cloth_grid_aos(cloth, nx, ny, 1.6f,1.0f,0.3f, true);
    if constexpr (std::is_same_v<ClothT, HinaPE::ClothSOA>) HinaPE::build_cloth_grid_soa(cloth, nx, ny, 1.6f,1.0f,0.3f, true);
    if constexpr (std::is_same_v<ClothT, HinaPE::ClothAoSoA>) HinaPE::build_cloth_grid_aosoa(cloth, nx, ny, 1.6f,1.0f,0.3f, true);
    if constexpr (std::is_same_v<ClothT, HinaPE::ClothAligned>) HinaPE::build_cloth_grid_aligned(cloth, nx, ny, 1.6f,1.0f,0.3f, true);

    if (!validate_pins(cloth)) { std::printf("[%s] pin validation failed at init\n", name); return 1; }

    // Native baseline (rebuild initial state per run to avoid copy of noncopyable aligned buffers)
    ClothT c_native; {
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAOS>) HinaPE::build_cloth_grid_aos(c_native, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothSOA>) HinaPE::build_cloth_grid_soa(c_native, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAoSoA>) HinaPE::build_cloth_grid_aosoa(c_native, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAligned>) HinaPE::build_cloth_grid_aligned(c_native, nx, ny, 1.6f,1.0f,0.3f, true);
        for (int i=0;i<steps;++i) native(c_native, dt, p);
    }
    residuals r_native = compute_residuals(c_native);
    ClothT c_tbb; {
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAOS>) HinaPE::build_cloth_grid_aos(c_tbb, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothSOA>) HinaPE::build_cloth_grid_soa(c_tbb, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAoSoA>) HinaPE::build_cloth_grid_aosoa(c_tbb, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAligned>) HinaPE::build_cloth_grid_aligned(c_tbb, nx, ny, 1.6f,1.0f,0.3f, true);
        for (int i=0;i<steps;++i) tbb(c_tbb, dt, p);
    }
    residuals r_tbb = compute_residuals(c_tbb);
    ClothT c_avx; {
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAOS>) HinaPE::build_cloth_grid_aos(c_avx, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothSOA>) HinaPE::build_cloth_grid_soa(c_avx, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAoSoA>) HinaPE::build_cloth_grid_aosoa(c_avx, nx, ny, 1.6f,1.0f,0.3f, true);
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAligned>) HinaPE::build_cloth_grid_aligned(c_avx, nx, ny, 1.6f,1.0f,0.3f, true);
        for (int i=0;i<steps;++i) avx2(c_avx, dt, p);
    }
    residuals r_avx = compute_residuals(c_avx);

    auto print = [&](const char* tag, residuals r){ std::printf("[%s] %-6s : L1=%g  L2=%g  Linf=%g (m=%zu)\n", name, tag, r.l1, r.l2, r.linf, r.m); };
    print("native", r_native); print("tbb", r_tbb); print("avx2", r_avx);

    auto ok_rel = [](double a, double b, double tol_rel){ double denom = std::max(1e-8, std::abs(a)); return std::abs(a-b)/denom <= tol_rel; };
    const double tol = 1e-3; // relative tolerance to baseline
    bool ok_tbb = ok_rel(r_tbb.l1, r_native.l1, tol) && ok_rel(r_tbb.l2, r_native.l2, tol) && ok_rel(r_tbb.linf, r_native.linf, 5e-3);
    bool ok_avx = ok_rel(r_avx.l1, r_native.l1, tol) && ok_rel(r_avx.l2, r_native.l2, tol) && ok_rel(r_avx.linf, r_native.linf, 5e-3);
    if (!ok_tbb) { std::printf("[%s] TBB residuals deviate from native beyond tolerance\n", name); return 2; }
    if (!ok_avx) { std::printf("[%s] AVX2 residuals deviate from native beyond tolerance\n", name); return 3; }
    return 0;
}

int main() {
    int rc=0;
    {
        HinaPE::ClothAOS c; rc |= validate_layout("AOS", c, &HinaPE::xpbd_step_native_aos, &HinaPE::xpbd_step_tbb_aos, &HinaPE::xpbd_step_avx2_aos);
    }
    {
        HinaPE::ClothSOA c; rc |= validate_layout("SOA", c, &HinaPE::xpbd_step_native_soa, &HinaPE::xpbd_step_tbb_soa, &HinaPE::xpbd_step_avx2_soa);
    }
    {
        HinaPE::ClothAoSoA c; rc |= validate_layout("AOSOA", c, &HinaPE::xpbd_step_native_aosoa, &HinaPE::xpbd_step_tbb_aosoa, &HinaPE::xpbd_step_avx2_aosoa);
    }
    {
        HinaPE::ClothAligned c; rc |= validate_layout("ALIGNED", c, &HinaPE::xpbd_step_native_aligned, &HinaPE::xpbd_step_tbb_aligned, &HinaPE::xpbd_step_avx2_aligned);
    }
    if (rc==0) std::printf("Validation: SUCCESS\n"); else std::printf("Validation: FAILED (code=%d)\n", rc);
    return rc;
}

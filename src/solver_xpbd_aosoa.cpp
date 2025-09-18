#include "aosoa/solver_xpbd_aosoa.h"
#include <algorithm>
#include <cmath>
#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif

namespace HinaPE {

static inline void index_to_block_lane_local(int idx, int& b, int& lane) { b = idx / AOSOA_BLOCK; lane = idx % AOSOA_BLOCK; }

void xpbd_step_native_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;

    // warmstart
    if (!params.warmstart) {
        for (auto& blk : cloth.cblocks) {
            for (int l=0;l<AOSOA_BLOCK;++l) blk.lambda[l] = 0.0f;
        }
    } else {
        for (auto& blk : cloth.cblocks) {
            for (int l=0;l<AOSOA_BLOCK;++l) blk.lambda[l] *= params.lambda_decay;
        }
    }

    int nb = (cloth.count + AOSOA_BLOCK - 1)/AOSOA_BLOCK;
    int ncb = (cloth.cons_count + AOSOA_BLOCK - 1)/AOSOA_BLOCK;

    for (int s = 0; s < substeps; ++s) {
        // predict
        for (int b=0;b<nb;++b) {
            auto& pb = cloth.pblocks[b];
            for (int l=0;l<AOSOA_BLOCK;++l) {
                pb.corr_x[l]=pb.corr_y[l]=pb.corr_z[l]=0.0f;
                if (pb.inv_mass[l] == 0.0f) { pb.vx[l]=pb.vy[l]=pb.vz[l]=0; pb.px[l]=pb.x[l]; pb.py[l]=pb.y[l]; pb.pz[l]=pb.z[l]; continue; }
                pb.vx[l]+=params.ax*h; pb.vy[l]+=params.ay*h; pb.vz[l]+=params.az*h;
                pb.px[l]=pb.x[l]; pb.py[l]=pb.y[l]; pb.pz[l]=pb.z[l];
                pb.x[l]+=pb.vx[l]*h; pb.y[l]+=pb.vy[l]*h; pb.z[l]+=pb.vz[l]*h;
            }
        }

        const float alpha_dt = 1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (int cb=0; cb<ncb; ++cb) {
                auto& blk = cloth.cblocks[cb];
                for (int l=0;l<AOSOA_BLOCK;++l) {
                    int ii = blk.i[l]; int jj = blk.j[l];
                    if (cb*AOSOA_BLOCK + l >= cloth.cons_count) break;
                    int bi,li; index_to_block_lane_local(ii,bi,li);
                    int bj,lj; index_to_block_lane_local(jj,bj,lj);
                    auto& pi = cloth.pblocks[bi];
                    auto& pj = cloth.pblocks[bj];
                    float dx = pi.x[li]-pj.x[lj];
                    float dy = pi.y[li]-pj.y[lj];
                    float dz = pi.z[li]-pj.z[lj];
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (dist < 1e-8f) { blk.last_c[l]=0; blk.last_dlambda[l]=0; blk.last_nx[l]=blk.last_ny[l]=blk.last_nz[l]=0; continue; }
                    float C = dist - blk.rest_length[l];
                    float nx = dx/dist, ny = dy/dist, nz = dz/dist;
                    float scale = params.compliance_scale_all;
                    switch (blk.type[l]) {
                        case ConstraintType::Structural: scale *= params.compliance_scale_structural; break;
                        case ConstraintType::Shear: scale *= params.compliance_scale_shear; break;
                        case ConstraintType::Bending: scale *= params.compliance_scale_bending; break;
                        default: break;
                    }
                    float alpha_tilde = (blk.compliance[l] * scale) * alpha_dt;
                    float wsum = pi.inv_mass[li] + pj.inv_mass[lj];
                    float denom = wsum + alpha_tilde;
                    if (denom <= 0.0f) { blk.last_c[l]=C; blk.last_dlambda[l]=0; blk.last_nx[l]=nx; blk.last_ny[l]=ny; blk.last_nz[l]=nz; continue; }
                    float dlambda = (-C - alpha_tilde * blk.lambda[l]) / denom;
                    blk.lambda[l] += dlambda;
                    float sx=dlambda*nx, sy=dlambda*ny, sz=dlambda*nz;
                    if (params.max_correction > 0.0f) {
                        float mag = std::sqrt(sx*sx + sy*sy + sz*sz);
                        if (mag > params.max_correction && mag > 0.0f) { float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }
                    }
                    if (pi.inv_mass[li]>0) { pi.x[li]+=pi.inv_mass[li]*sx; pi.y[li]+=pi.inv_mass[li]*sy; pi.z[li]+=pi.inv_mass[li]*sz; pi.corr_x[li]+=pi.inv_mass[li]*sx; pi.corr_y[li]+=pi.inv_mass[li]*sy; pi.corr_z[li]+=pi.inv_mass[li]*sz; }
                    if (pj.inv_mass[lj]>0) { pj.x[lj]-=pj.inv_mass[lj]*sx; pj.y[lj]-=pj.inv_mass[lj]*sy; pj.z[lj]-=pj.inv_mass[lj]*sz; pj.corr_x[lj]-=pj.inv_mass[lj]*sx; pj.corr_y[lj]-=pj.inv_mass[lj]*sy; pj.corr_z[lj]-=pj.inv_mass[lj]*sz; }
                    if (params.write_debug_fields) { blk.last_c[l]=C; blk.last_dlambda[l]=dlambda; blk.last_nx[l]=nx; blk.last_ny[l]=ny; blk.last_nz[l]=nz; }
                }
            }
        }

        // velocities
        float inv_h = 1.0f/h;
        for (int b=0;b<nb;++b) {
            auto& pb = cloth.pblocks[b];
            for (int l=0;l<AOSOA_BLOCK;++l) {
                pb.vx[l]=(pb.x[l]-pb.px[l])*inv_h;
                pb.vy[l]=(pb.y[l]-pb.py[l])*inv_h;
                pb.vz[l]=(pb.z[l]-pb.pz[l])*inv_h;
            }
        }
        if (params.velocity_damping>0.0f) {
            float s = std::max(0.0f, 1.0f-params.velocity_damping);
            for (int b=0;b<nb;++b) {
                auto& pb = cloth.pblocks[b];
                for (int l=0;l<AOSOA_BLOCK;++l) { pb.vx[l]*=s; pb.vy[l]*=s; pb.vz[l]*=s; }
            }
        }
    }

    cloth.last_dt = clamped_dt; cloth.last_iterations = params.iterations;
}

void xpbd_step_tbb_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    // Parallelize predict/velocity by blocks; constraints remain native
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    int nb = (cloth.count + AOSOA_BLOCK - 1)/AOSOA_BLOCK; int ncb=(cloth.cons_count + AOSOA_BLOCK - 1)/AOSOA_BLOCK;
    if (!params.warmstart) for (auto& cb: cloth.cblocks) for (int l=0;l<AOSOA_BLOCK;++l) cb.lambda[l]=0.0f; else for (auto& cb: cloth.cblocks) for (int l=0;l<AOSOA_BLOCK;++l) cb.lambda[l]*=params.lambda_decay;
    for (int s=0;s<substeps;++s){
        tbb::parallel_for(0, nb, [&](int b){ auto& pb=cloth.pblocks[b]; for (int l=0;l<AOSOA_BLOCK;++l){ pb.corr_x[l]=pb.corr_y[l]=pb.corr_z[l]=0.0f; if (pb.inv_mass[l]==0.0f){ pb.vx[l]=pb.vy[l]=pb.vz[l]=0; pb.px[l]=pb.x[l]; pb.py[l]=pb.y[l]; pb.pz[l]=pb.z[l]; } else { pb.vx[l]+=params.ax*h; pb.vy[l]+=params.ay*h; pb.vz[l]+=params.az*h; pb.px[l]=pb.x[l]; pb.py[l]=pb.y[l]; pb.pz[l]=pb.z[l]; pb.x[l]+=pb.vx[l]*h; pb.y[l]+=pb.vy[l]*h; pb.z[l]+=pb.vz[l]*h; } }});
        const float alpha_dt=1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it){
            for (int cb=0; cb<ncb; ++cb){ auto& blk=cloth.cblocks[cb]; for (int l=0;l<AOSOA_BLOCK;++l){ int k=cb*AOSOA_BLOCK + l; if (k>=cloth.cons_count) break; int ii=blk.i[l], jj=blk.j[l]; int bi=ii/AOSOA_BLOCK, li=ii% AOSOA_BLOCK; int bj=jj/AOSOA_BLOCK, lj=jj% AOSOA_BLOCK; auto& pi=cloth.pblocks[bi]; auto& pj=cloth.pblocks[bj]; float dx=pi.x[li]-pj.x[lj], dy=pi.y[li]-pj.y[lj], dz=pi.z[li]-pj.z[lj]; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if(params.write_debug_fields){ blk.last_c[l]=0; blk.last_dlambda[l]=0; blk.last_nx[l]=blk.last_ny[l]=blk.last_nz[l]=0;} continue; } float C=dist - blk.rest_length[l]; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(blk.type[l]){ case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;} float alpha_tilde=(blk.compliance[l]*scale)*alpha_dt; float wsum=pi.inv_mass[li]+pj.inv_mass[lj]; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ blk.last_c[l]=C; blk.last_dlambda[l]=0; blk.last_nx[l]=nx; blk.last_ny[l]=ny; blk.last_nz[l]=nz;} continue; } float dl=(-C - alpha_tilde*blk.lambda[l])/denom; blk.lambda[l]+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; } } if (pi.inv_mass[li]>0){ pi.x[li]+=pi.inv_mass[li]*sx; pi.y[li]+=pi.inv_mass[li]*sy; pi.z[li]+=pi.inv_mass[li]*sz; } if (pj.inv_mass[lj]>0){ pj.x[lj]-=pj.inv_mass[lj]*sx; pj.y[lj]-=pj.inv_mass[lj]*sy; pj.z[lj]-=pj.inv_mass[lj]*sz; } if (params.write_debug_fields){ blk.last_c[l]=C; blk.last_dlambda[l]=dl; blk.last_nx[l]=nx; blk.last_ny[l]=ny; blk.last_nz[l]=nz; } }}
        }
        tbb::parallel_for(0, nb, [&](int b){ auto& pb=cloth.pblocks[b]; for (int l=0;l<AOSOA_BLOCK;++l){ float inv_h=1.0f/h; pb.vx[l]=(pb.x[l]-pb.px[l])*inv_h; pb.vy[l]=(pb.y[l]-pb.py[l])*inv_h; pb.vz[l]=(pb.z[l]-pb.pz[l])*inv_h; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); pb.vx[l]*=s; pb.vy[l]*=s; pb.vz[l]*=s; } }});
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_aosoa(cloth, dt, params);
#endif
}

void xpbd_step_avx2_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
    // For AoSoA, keep native (AoSoA already cache-friendly). Advanced AVX2 would require lane-wise packing.
    xpbd_step_native_aosoa(cloth, dt, params);
}

} // namespace HinaPE

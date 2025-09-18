#include "aligned/solver_xpbd_aligned.h"
#include <algorithm>
#include <cmath>
#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace HinaPE {

void xpbd_step_native_aligned(ClothAligned& cloth, float dt, const XPBDParams& params) {
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;

    const int n = (int)cloth.x.size;
    const int m = (int)cloth.ci.size;

    if (!params.warmstart) {
        for (int k=0;k<m;++k) cloth.lambda[k]=0.0f;
    } else {
        for (int k=0;k<m;++k) cloth.lambda[k]*=params.lambda_decay;
    }

    for (int s=0;s<substeps;++s) {
        // predict
        for (int i=0;i<n;++i) {
            if (cloth.inv_mass[i] == 0.0f) { cloth.vx[i]=cloth.vy[i]=cloth.vz[i]=0; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; continue; }
            cloth.vx[i]+=params.ax*h; cloth.vy[i]+=params.ay*h; cloth.vz[i]+=params.az*h;
            cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i];
            cloth.x[i]+=cloth.vx[i]*h; cloth.y[i]+=cloth.vy[i]*h; cloth.z[i]+=cloth.vz[i]*h;
        }

        // constraints (iterations)
        const float alpha_dt = 1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (int k=0;k<m;++k) {
                int i = cloth.ci[k], j = cloth.cj[k];
                float dx = cloth.x[i]-cloth.x[j];
                float dy = cloth.y[i]-cloth.y[j];
                float dz = cloth.z[i]-cloth.z[j];
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-8f) { if (params.write_debug_fields){ cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0;} continue; }
                float C = dist - cloth.rest_length[k];
                float nx = dx/dist, ny=dy/dist, nz=dz/dist;
                float scale = params.compliance_scale_all;
                switch (cloth.type[k]) {
                    case ConstraintType::Structural: scale *= params.compliance_scale_structural; break;
                    case ConstraintType::Shear:      scale *= params.compliance_scale_shear; break;
                    case ConstraintType::Bending:    scale *= params.compliance_scale_bending; break;
                    default: break;
                }
                float alpha_tilde = (cloth.compliance[k] * scale) * alpha_dt;
                float wsum = cloth.inv_mass[i] + cloth.inv_mass[j];
                float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) { if (params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz;} continue; }
                float dlambda = (-C - alpha_tilde * cloth.lambda[k]) / denom;
                cloth.lambda[k] += dlambda;
                float sx=dlambda*nx, sy=dlambda*ny, sz=dlambda*nz;
                if (params.max_correction > 0.0f) {
                    float mag = std::sqrt(sx*sx + sy*sy + sz*sz);
                    if (mag > params.max_correction && mag > 0.0f) { float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }
                }
                if (cloth.inv_mass[i]>0) { cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; }
                if (cloth.inv_mass[j]>0) { cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; }
                if (params.write_debug_fields) { cloth.last_c[k]=C; cloth.last_dlambda[k]=dlambda; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; }
            }
        }

        // velocities
        float inv_h = 1.0f/h;
        for (int i=0;i<n;++i) {
            cloth.vx[i]=(cloth.x[i]-cloth.px[i])*inv_h;
            cloth.vy[i]=(cloth.y[i]-cloth.py[i])*inv_h;
            cloth.vz[i]=(cloth.z[i]-cloth.pz[i])*inv_h;
        }
        if (params.velocity_damping>0.0f) {
            float s = std::max(0.0f, 1.0f-params.velocity_damping);
            for (int i=0;i<n;++i) { cloth.vx[i]*=s; cloth.vy[i]*=s; cloth.vz[i]*=s; }
        }
    }

    cloth.last_dt = clamped_dt; cloth.last_iterations = params.iterations;
}

void xpbd_step_tbb_aligned(ClothAligned& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    const int n=(int)cloth.x.size; const int m=(int)cloth.ci.size;
    if (!params.warmstart) for (int k=0;k<m;++k) cloth.lambda[k]=0.0f; else for (int k=0;k<m;++k) cloth.lambda[k]*=params.lambda_decay;
    for (int s=0;s<substeps;++s){
        tbb::parallel_for(0, n, [&](int i){ if (cloth.inv_mass[i]==0.0f){ cloth.vx[i]=cloth.vy[i]=cloth.vz[i]=0; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; } else { cloth.vx[i]+=params.ax*h; cloth.vy[i]+=params.ay*h; cloth.vz[i]+=params.az*h; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; cloth.x[i]+=cloth.vx[i]*h; cloth.y[i]+=cloth.vy[i]*h; cloth.z[i]+=cloth.vz[i]*h; }});
        const float alpha_dt=1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it){ for (int k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; float dx=cloth.x[i]-cloth.x[j], dy=cloth.y[i]-cloth.y[j], dz=cloth.z[i]-cloth.z[j]; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if(params.write_debug_fields){ cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0; } continue; } float C=dist-cloth.rest_length[k]; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(cloth.type[k]){ case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;} float alpha_tilde=(cloth.compliance[k]*scale)*alpha_dt; float wsum=cloth.inv_mass[i]+cloth.inv_mass[j]; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz;} continue; } float dl=(-C - alpha_tilde*cloth.lambda[k])/denom; cloth.lambda[k]+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; } } if (cloth.inv_mass[i]>0){ cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; } if (cloth.inv_mass[j]>0){ cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; } if (params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=dl; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; } } }
        tbb::parallel_for(0, n, [&](int i){ float inv_h=1.0f/h; cloth.vx[i]=(cloth.x[i]-cloth.px[i])*inv_h; cloth.vy[i]=(cloth.y[i]-cloth.py[i])*inv_h; cloth.vz[i]=(cloth.z[i]-cloth.pz[i])*inv_h; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); cloth.vx[i]*=s; cloth.vy[i]*=s; cloth.vz[i]*=s; } });
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_aligned(cloth, dt, params);
#endif
}

void xpbd_step_avx2_aligned(ClothAligned& cloth, float dt, const XPBDParams& params) {
#if defined(__AVX2__)
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    const int n=(int)cloth.x.size; const int m=(int)cloth.ci.size;
    if (!params.warmstart) for (int k=0;k<m;++k) cloth.lambda[k]=0.0f; else for (int k=0;k<m;++k) cloth.lambda[k]*=params.lambda_decay;
    __m256 vax=_mm256_set1_ps(params.ax*h), vay=_mm256_set1_ps(params.ay*h), vaz=_mm256_set1_ps(params.az*h), vh=_mm256_set1_ps(h), invh_const=_mm256_set1_ps(1.0f/h);
    for (int s=0;s<substeps;++s){
        int i=0; for (; i+8<=n; i+=8){ __m256 invm=_mm256_loadu_ps(&cloth.inv_mass[i]); __m256 zero=_mm256_set1_ps(0.0f); __m256 mask=_mm256_cmp_ps(invm, zero, _CMP_NEQ_OQ);
            __m256 mvx=_mm256_load_ps(&cloth.vx[i]), mvy=_mm256_load_ps(&cloth.vy[i]), mvz=_mm256_load_ps(&cloth.vz[i]);
            mvx=_mm256_blendv_ps(mvx, _mm256_add_ps(mvx,vax), mask); mvy=_mm256_blendv_ps(mvy, _mm256_add_ps(mvy,vay), mask); mvz=_mm256_blendv_ps(mvz, _mm256_add_ps(mvz,vaz), mask);
            _mm256_store_ps(&cloth.px[i], _mm256_load_ps(&cloth.x[i])); _mm256_store_ps(&cloth.py[i], _mm256_load_ps(&cloth.y[i])); _mm256_store_ps(&cloth.pz[i], _mm256_load_ps(&cloth.z[i]));
            _mm256_store_ps(&cloth.vx[i], mvx); _mm256_store_ps(&cloth.vy[i], mvy); _mm256_store_ps(&cloth.vz[i], mvz);
            __m256 mx=_mm256_add_ps(_mm256_load_ps(&cloth.x[i]), _mm256_mul_ps(mvx, vh)); __m256 my=_mm256_add_ps(_mm256_load_ps(&cloth.y[i]), _mm256_mul_ps(mvy, vh)); __m256 mz=_mm256_add_ps(_mm256_load_ps(&cloth.z[i]), _mm256_mul_ps(mvz, vh));
            _mm256_store_ps(&cloth.x[i], mx); _mm256_store_ps(&cloth.y[i], my); _mm256_store_ps(&cloth.z[i], mz);
        }
        for (; i<n; ++i){ if (cloth.inv_mass[i]==0.0f){ cloth.vx[i]=cloth.vy[i]=cloth.vz[i]=0; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; } else { cloth.vx[i]+=params.ax*h; cloth.vy[i]+=params.ay*h; cloth.vz[i]+=params.az*h; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; cloth.x[i]+=cloth.vx[i]*h; cloth.y[i]+=cloth.vy[i]*h; cloth.z[i]+=cloth.vz[i]*h; } }
        // native constraints
        const float alpha_dt=1.0f/(h*h); for (int it=0; it<std::max(1, params.iterations); ++it){ for (int k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; float dx=cloth.x[i]-cloth.x[j], dy=cloth.y[i]-cloth.y[j], dz=cloth.z[i]-cloth.z[j]; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if (params.write_debug_fields){ cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0;} continue; } float C=dist-cloth.rest_length[k]; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(cloth.type[k]){ case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;} float alpha_tilde=(cloth.compliance[k]*scale)*alpha_dt; float wsum=cloth.inv_mass[i]+cloth.inv_mass[j]; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz;} continue; } float dl=(-C - alpha_tilde*cloth.lambda[k])/denom; cloth.lambda[k]+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }} if (cloth.inv_mass[i]>0){ cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; } if (cloth.inv_mass[j]>0){ cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; } if (params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=dl; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; } } }
        // velocities
        int j=0; for (; j+8<=n; j+=8){ __m256 mx=_mm256_load_ps(&cloth.x[j]), my=_mm256_load_ps(&cloth.y[j]), mz=_mm256_load_ps(&cloth.z[j]); __m256 mpx=_mm256_load_ps(&cloth.px[j]), mpy=_mm256_load_ps(&cloth.py[j]), mpz=_mm256_load_ps(&cloth.pz[j]); __m256 mvx=_mm256_mul_ps(_mm256_sub_ps(mx,mpx),invh_const); __m256 mvy=_mm256_mul_ps(_mm256_sub_ps(my,mpy),invh_const); __m256 mvz=_mm256_mul_ps(_mm256_sub_ps(mz,mpz),invh_const); if (params.velocity_damping>0.0f){ __m256 s=_mm256_set1_ps(std::max(0.0f,1.0f-params.velocity_damping)); mvx=_mm256_mul_ps(mvx,s); mvy=_mm256_mul_ps(mvy,s); mvz=_mm256_mul_ps(mvz,s);} _mm256_store_ps(&cloth.vx[j], mvx); _mm256_store_ps(&cloth.vy[j], mvy); _mm256_store_ps(&cloth.vz[j], mvz); }
        for (; j<n; ++j){ float invh=1.0f/h; cloth.vx[j]=(cloth.x[j]-cloth.px[j])*invh; cloth.vy[j]=(cloth.y[j]-cloth.py[j])*invh; cloth.vz[j]=(cloth.z[j]-cloth.pz[j])*invh; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); cloth.vx[j]*=s; cloth.vy[j]*=s; cloth.vz[j]*=s; } }
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_aligned(cloth, dt, params);
#endif
}

} // namespace HinaPE

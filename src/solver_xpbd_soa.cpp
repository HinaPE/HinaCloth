#include "soa/solver_xpbd_soa.h"
#include <algorithm>
#include <cmath>
#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace HinaPE {

void xpbd_step_native_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;

    if (!params.warmstart) {
        std::fill(cloth.lambda.begin(), cloth.lambda.end(), 0.0f);
    } else {
        for (auto& l : cloth.lambda) l *= params.lambda_decay;
    }

    int n = (int)cloth.x.size();
    for (int s = 0; s < substeps; ++s) {
        // predict
        std::fill(cloth.corr_x.begin(), cloth.corr_x.end(), 0.0f);
        std::fill(cloth.corr_y.begin(), cloth.corr_y.end(), 0.0f);
        std::fill(cloth.corr_z.begin(), cloth.corr_z.end(), 0.0f);
        for (int i=0;i<n;++i) {
            if (cloth.inv_mass[i] == 0.0f) { cloth.vx[i]=cloth.vy[i]=cloth.vz[i]=0; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; continue; }
            cloth.vx[i] += params.ax*h; cloth.vy[i] += params.ay*h; cloth.vz[i] += params.az*h;
            cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i];
            cloth.x[i] += cloth.vx[i]*h; cloth.y[i] += cloth.vy[i]*h; cloth.z[i] += cloth.vz[i]*h;
        }

        // constraints
        const float alpha_dt = 1.0f/(h*h);
        int m = (int)cloth.ci.size();
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (int k=0;k<m;++k) {
                int i = cloth.ci[k]; int j = cloth.cj[k];
                float dx = cloth.x[i]-cloth.x[j];
                float dy = cloth.y[i]-cloth.y[j];
                float dz = cloth.z[i]-cloth.z[j];
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-8f) { cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0; continue; }
                float C = dist - cloth.rest_length[k];
                float nx = dx/dist, ny = dy/dist, nz = dz/dist;
                float scale = params.compliance_scale_all;
                switch (cloth.type[k]) {
                    case ConstraintType::Structural: scale *= params.compliance_scale_structural; break;
                    case ConstraintType::Shear: scale *= params.compliance_scale_shear; break;
                    case ConstraintType::Bending: scale *= params.compliance_scale_bending; break;
                    default: break;
                }
                float alpha_tilde = (cloth.compliance[k] * scale) * alpha_dt;
                float wsum = cloth.inv_mass[i] + cloth.inv_mass[j];
                float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) { cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; continue; }
                float dlambda = (-C - alpha_tilde * cloth.lambda[k]) / denom;
                cloth.lambda[k] += dlambda;
                float sx = dlambda*nx, sy=dlambda*ny, sz=dlambda*nz;
                if (params.max_correction > 0.0f) {
                    float mag = std::sqrt(sx*sx + sy*sy + sz*sz);
                    if (mag > params.max_correction && mag > 0.0f) { float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }
                }
                if (cloth.inv_mass[i]>0) { cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; cloth.corr_x[i]+=cloth.inv_mass[i]*sx; cloth.corr_y[i]+=cloth.inv_mass[i]*sy; cloth.corr_z[i]+=cloth.inv_mass[i]*sz; }
                if (cloth.inv_mass[j]>0) { cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; cloth.corr_x[j]-=cloth.inv_mass[j]*sx; cloth.corr_y[j]-=cloth.inv_mass[j]*sy; cloth.corr_z[j]-=cloth.inv_mass[j]*sz; }
                if (params.write_debug_fields) { cloth.last_c[k]=C; cloth.last_dlambda[k]=dlambda; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; }
            }
        }

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

void xpbd_step_tbb_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    if (!params.warmstart) std::fill(cloth.lambda.begin(), cloth.lambda.end(), 0.0f); else for (auto& l: cloth.lambda) l*=params.lambda_decay;
    int n=(int)cloth.x.size(); int m=(int)cloth.ci.size();
    for (int s=0;s<substeps;++s){
        std::fill(cloth.corr_x.begin(), cloth.corr_x.end(), 0.0f);
        std::fill(cloth.corr_y.begin(), cloth.corr_y.end(), 0.0f);
        std::fill(cloth.corr_z.begin(), cloth.corr_z.end(), 0.0f);
        tbb::parallel_for(0, n, [&](int i){ if (cloth.inv_mass[i]==0.0f){ cloth.vx[i]=cloth.vy[i]=cloth.vz[i]=0; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; } else { cloth.vx[i]+=params.ax*h; cloth.vy[i]+=params.ay*h; cloth.vz[i]+=params.az*h; cloth.px[i]=cloth.x[i]; cloth.py[i]=cloth.y[i]; cloth.pz[i]=cloth.z[i]; cloth.x[i]+=cloth.vx[i]*h; cloth.y[i]+=cloth.vy[i]*h; cloth.z[i]+=cloth.vz[i]*h; } });
        const float alpha_dt=1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it){
            for (int k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; float dx=cloth.x[i]-cloth.x[j]; float dy=cloth.y[i]-cloth.y[j]; float dz=cloth.z[i]-cloth.z[j]; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if(params.write_debug_fields){ cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0; } continue; } float C=dist-cloth.rest_length[k]; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(cloth.type[k]){ case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;} float alpha_tilde=(cloth.compliance[k]*scale)*alpha_dt; float wsum=cloth.inv_mass[i]+cloth.inv_mass[j]; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz;} continue; } float dl=(-C - alpha_tilde*cloth.lambda[k])/denom; cloth.lambda[k]+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; } } if (cloth.inv_mass[i]>0){ cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; } if (cloth.inv_mass[j]>0){ cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; } if (params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=dl; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; } }
        }
        tbb::parallel_for(0, n, [&](int i){ float inv_h=1.0f/h; cloth.vx[i]=(cloth.x[i]-cloth.px[i])*inv_h; cloth.vy[i]=(cloth.y[i]-cloth.py[i])*inv_h; cloth.vz[i]=(cloth.z[i]-cloth.pz[i])*inv_h; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); cloth.vx[i]*=s; cloth.vy[i]*=s; cloth.vz[i]*=s; } });
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_soa(cloth, dt, params);
#endif
}

void xpbd_step_avx2_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
#if defined(__AVX2__)
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    if (!params.warmstart) std::fill(cloth.lambda.begin(), cloth.lambda.end(), 0.0f); else for (auto& l: cloth.lambda) l*=params.lambda_decay;
    int n=(int)cloth.x.size(); int m=(int)cloth.ci.size();
    __m256 vax=_mm256_set1_ps(params.ax*h), vay=_mm256_set1_ps(params.ay*h), vaz=_mm256_set1_ps(params.az*h), vh=_mm256_set1_ps(h);
    for (int s=0;s<substeps;++s){
        // predict vectorized
        for (int i=0;i<n;i+=8){ int rest=std::min(8, n-i); float invm[8]={0}; for(int k=0;k<rest;++k) invm[k]=cloth.inv_mass[i+k]; __m256 mInv=_mm256_loadu_ps(invm); __m256 zero=_mm256_set1_ps(0.0f); __m256 mask=_mm256_cmp_ps(mInv, zero, _CMP_NEQ_OQ);
            float vx[8],vy[8],vz[8],x[8],y[8],z[8],px[8],py[8],pz[8]; for(int k=0;k<rest;++k){ vx[k]=cloth.vx[i+k]; vy[k]=cloth.vy[i+k]; vz[k]=cloth.vz[i+k]; x[k]=cloth.x[i+k]; y[k]=cloth.y[i+k]; z[k]=cloth.z[i+k]; px[k]=cloth.px[i+k]=cloth.x[i+k]; py[k]=cloth.py[i+k]=cloth.y[i+k]; pz[k]=cloth.pz[i+k]=cloth.z[i+k]; }
            __m256 mvx=_mm256_loadu_ps(vx), mvy=_mm256_loadu_ps(vy), mvz=_mm256_loadu_ps(vz), mx=_mm256_loadu_ps(x), my=_mm256_loadu_ps(y), mz=_mm256_loadu_ps(z);
            mvx=_mm256_blendv_ps(mvx, _mm256_add_ps(mvx,vax), mask); mvy=_mm256_blendv_ps(mvy, _mm256_add_ps(mvy,vay), mask); mvz=_mm256_blendv_ps(mvz, _mm256_add_ps(mvz,vaz), mask);
            mx=_mm256_add_ps(mx, _mm256_mul_ps(mvx,vh)); my=_mm256_add_ps(my, _mm256_mul_ps(mvy,vh)); mz=_mm256_add_ps(mz, _mm256_mul_ps(mvz,vh)); _mm256_storeu_ps(vx,mvx); _mm256_storeu_ps(vy,mvy); _mm256_storeu_ps(vz,mvz); _mm256_storeu_ps(x,mx); _mm256_storeu_ps(y,my); _mm256_storeu_ps(z,mz);
            for(int k=0;k<rest;++k){ if (cloth.inv_mass[i+k]!=0.0f){ cloth.vx[i+k]=vx[k]; cloth.vy[i+k]=vy[k]; cloth.vz[i+k]=vz[k]; cloth.x[i+k]=x[k]; cloth.y[i+k]=y[k]; cloth.z[i+k]=z[k]; } }
        }
        // constraints native
        const float alpha_dt=1.0f/(h*h); for (int it=0; it<std::max(1, params.iterations); ++it){ for (int k=0;k<m;++k){ int i=cloth.ci[k], j=cloth.cj[k]; float dx=cloth.x[i]-cloth.x[j]; float dy=cloth.y[i]-cloth.y[j]; float dz=cloth.z[i]-cloth.z[j]; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if(params.write_debug_fields){ cloth.last_c[k]=0; cloth.last_dlambda[k]=0; cloth.last_nx[k]=cloth.last_ny[k]=cloth.last_nz[k]=0;} continue; } float C=dist-cloth.rest_length[k]; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(cloth.type[k]){ case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;} float alpha_tilde=(cloth.compliance[k]*scale)*alpha_dt; float wsum=cloth.inv_mass[i]+cloth.inv_mass[j]; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=0; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz;} continue; } float dl=(-C - alpha_tilde*cloth.lambda[k])/denom; cloth.lambda[k]+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; } } if (cloth.inv_mass[i]>0){ cloth.x[i]+=cloth.inv_mass[i]*sx; cloth.y[i]+=cloth.inv_mass[i]*sy; cloth.z[i]+=cloth.inv_mass[i]*sz; } if (cloth.inv_mass[j]>0){ cloth.x[j]-=cloth.inv_mass[j]*sx; cloth.y[j]-=cloth.inv_mass[j]*sy; cloth.z[j]-=cloth.inv_mass[j]*sz; } if (params.write_debug_fields){ cloth.last_c[k]=C; cloth.last_dlambda[k]=dl; cloth.last_nx[k]=nx; cloth.last_ny[k]=ny; cloth.last_nz[k]=nz; } } }
        // velocities vectorized
        __m256 invh=_mm256_set1_ps(1.0f/h); for (int i=0;i<n;i+=8){ int rest=std::min(8,n-i); float x[8],y[8],z[8],px[8],py[8],pz[8]; for(int k=0;k<rest;++k){ x[k]=cloth.x[i+k]; y[k]=cloth.y[i+k]; z[k]=cloth.z[i+k]; px[k]=cloth.px[i+k]; py[k]=cloth.py[i+k]; pz[k]=cloth.pz[i+k]; }
            __m256 mx=_mm256_loadu_ps(x), my=_mm256_loadu_ps(y), mz=_mm256_loadu_ps(z), mpx=_mm256_loadu_ps(px), mpy=_mm256_loadu_ps(py), mpz=_mm256_loadu_ps(pz);
            __m256 mvx=_mm256_mul_ps(_mm256_sub_ps(mx,mpx), invh); __m256 mvy=_mm256_mul_ps(_mm256_sub_ps(my,mpy), invh); __m256 mvz=_mm256_mul_ps(_mm256_sub_ps(mz,mpz), invh);
            _mm256_storeu_ps(x, mvx); _mm256_storeu_ps(y, mvy); _mm256_storeu_ps(z, mvz); for(int k=0;k<rest;++k){ cloth.vx[i+k]=x[k]; cloth.vy[i+k]=y[k]; cloth.vz[i+k]=z[k]; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); cloth.vx[i+k]*=s; cloth.vy[i+k]*=s; cloth.vz[i+k]*=s; } }
        }
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_soa(cloth, dt, params);
#endif
}

} // namespace HinaPE

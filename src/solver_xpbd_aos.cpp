#include "aos/solver_xpbd_aos.h"
#include <algorithm>
#include <cmath>
#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace HinaPE {

void xpbd_step_native_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;

    if (!params.warmstart) {
        for (auto& c : cloth.constraints) c.lambda = 0.0f;
    } else {
        for (auto& c : cloth.constraints) c.lambda *= params.lambda_decay;
    }

    for (int s = 0; s < substeps; ++s) {
        // predict
        for (auto& p : cloth.particles) {
            p.corr_x = p.corr_y = p.corr_z = 0.0f;
            if (p.inv_mass == 0.0f) { p.vx=0; p.vy=0; p.vz=0; p.px=p.x; p.py=p.y; p.pz=p.z; continue; }
            p.vx += params.ax * h; p.vy += params.ay * h; p.vz += params.az * h;
            p.px=p.x; p.py=p.y; p.pz=p.z;
            p.x += p.vx * h; p.y += p.vy * h; p.z += p.vz * h;
        }

        // constraints
        const float alpha_dt = 1.0f / (h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (auto& c : cloth.constraints) {
                auto& pi = cloth.particles[c.i];
                auto& pj = cloth.particles[c.j];
                float dx = pi.x - pj.x, dy = pi.y - pj.y, dz = pi.z - pj.z;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-8f) { c.last_c=0; c.last_dlambda=0; c.last_nx=c.last_ny=c.last_nz=0; continue; }
                float C = dist - c.rest_length;
                float nx = dx/dist, ny = dy/dist, nz = dz/dist;
                float scale = params.compliance_scale_all;
                switch (c.type) {
                    case ConstraintType::Structural: scale *= params.compliance_scale_structural; break;
                    case ConstraintType::Shear: scale *= params.compliance_scale_shear; break;
                    case ConstraintType::Bending: scale *= params.compliance_scale_bending; break;
                    default: break;
                }
                float alpha_tilde = (c.compliance * scale) * alpha_dt;
                float wsum = pi.inv_mass + pj.inv_mass;
                float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) { c.last_c=C; c.last_dlambda=0; c.last_nx=nx; c.last_ny=ny; c.last_nz=nz; continue; }
                float dlambda = (-C - alpha_tilde * c.lambda) / denom;
                c.lambda += dlambda;
                float sx = dlambda*nx, sy = dlambda*ny, sz = dlambda*nz;
                if (params.max_correction > 0.0f) {
                    float mag = std::sqrt(sx*sx + sy*sy + sz*sz);
                    if (mag > params.max_correction && mag > 0.0f) { float r = params.max_correction/mag; sx*=r; sy*=r; sz*=r; }
                }
                if (pi.inv_mass>0) { pi.x += pi.inv_mass*sx; pi.y += pi.inv_mass*sy; pi.z += pi.inv_mass*sz; pi.corr_x += pi.inv_mass*sx; pi.corr_y += pi.inv_mass*sy; pi.corr_z += pi.inv_mass*sz; }
                if (pj.inv_mass>0) { pj.x -= pj.inv_mass*sx; pj.y -= pj.inv_mass*sy; pj.z -= pj.inv_mass*sz; pj.corr_x -= pj.inv_mass*sx; pj.corr_y -= pj.inv_mass*sy; pj.corr_z -= pj.inv_mass*sz; }
                if (params.write_debug_fields) { c.last_c=C; c.last_dlambda=dlambda; c.last_nx=nx; c.last_ny=ny; c.last_nz=nz; }
            }
        }

        // velocities
        float inv_h = 1.0f / h;
        for (auto& p : cloth.particles) {
            p.vx = (p.x - p.px)*inv_h; p.vy = (p.y - p.py)*inv_h; p.vz = (p.z - p.pz)*inv_h;
        }
        if (params.velocity_damping > 0.0f) {
            float s = std::max(0.0f, 1.0f - params.velocity_damping);
            for (auto& p : cloth.particles) { p.vx*=s; p.vy*=s; p.vz*=s; }
        }
    }

    cloth.last_dt = clamped_dt;
    cloth.last_iterations = params.iterations;
}

void xpbd_step_tbb_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
    // Parallelize predict/velocity phases with TBB; constraints remain native (GS)
#if defined(HINACLOTH_HAVE_TBB)
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    if (!params.warmstart) for (auto& c: cloth.constraints) c.lambda = 0.0f; else for (auto& c: cloth.constraints) c.lambda *= params.lambda_decay;
    for (int s=0;s<substeps;++s) {
        tbb::parallel_for(std::size_t(0), cloth.particles.size(), [&](std::size_t idx){
            auto& p = cloth.particles[idx];
            p.corr_x=p.corr_y=p.corr_z=0.0f;
            if (p.inv_mass==0.0f) { p.vx=p.vy=p.vz=0; p.px=p.x; p.py=p.y; p.pz=p.z; return; }
            p.vx += params.ax*h; p.vy+=params.ay*h; p.vz+=params.az*h;
            p.px=p.x; p.py=p.y; p.pz=p.z; p.x+=p.vx*h; p.y+=p.vy*h; p.z+=p.vz*h;
        });
        // native constraints
        const float alpha_dt = 1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (auto& c : cloth.constraints) {
                auto& pi = cloth.particles[c.i]; auto& pj = cloth.particles[c.j];
                float dx=pi.x-pj.x, dy=pi.y-pj.y, dz=pi.z-pj.z; float dist=std::sqrt(dx*dx+dy*dy+dz*dz);
                if (dist<1e-8f) { if (params.write_debug_fields){ c.last_c=0; c.last_dlambda=0; c.last_nx=c.last_ny=c.last_nz=0;} continue; }
                float C=dist-c.rest_length; float nx=dx/dist, ny=dy/dist, nz=dz/dist;
                float scale=params.compliance_scale_all; switch(c.type){case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;}
                float alpha_tilde=(c.compliance*scale)*alpha_dt; float wsum=pi.inv_mass+pj.inv_mass; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if (params.write_debug_fields){ c.last_c=C; c.last_dlambda=0; c.last_nx=nx; c.last_ny=ny; c.last_nz=nz;} continue; }
                float dl=(-C - alpha_tilde*c.lambda)/denom; c.lambda+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }}
                if (pi.inv_mass>0){ pi.x+=pi.inv_mass*sx; pi.y+=pi.inv_mass*sy; pi.z+=pi.inv_mass*sz; }
                if (pj.inv_mass>0){ pj.x-=pj.inv_mass*sx; pj.y-=pj.inv_mass*sy; pj.z-=pj.inv_mass*sz; }
                if (params.write_debug_fields){ c.last_c=C; c.last_dlambda=dl; c.last_nx=nx; c.last_ny=ny; c.last_nz=nz; }
            }
        }
        tbb::parallel_for(std::size_t(0), cloth.particles.size(), [&](std::size_t idx){
            auto& p = cloth.particles[idx]; float inv_h=1.0f/h; p.vx=(p.x-p.px)*inv_h; p.vy=(p.y-p.py)*inv_h; p.vz=(p.z-p.pz)*inv_h; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); p.vx*=s; p.vy*=s; p.vz*=s; }
        });
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_aos(cloth, dt, params);
#endif
}

void xpbd_step_avx2_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
#if defined(__AVX2__)
    // AVX2-accelerated predict/velocity phases; constraints remain native
    float clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    int substeps = std::max(1, params.substeps);
    float h = clamped_dt / (float)substeps;
    if (!params.warmstart) for (auto& c: cloth.constraints) c.lambda=0.0f; else for (auto& c: cloth.constraints) c.lambda*=params.lambda_decay;
    __m256 vax=_mm256_set1_ps(params.ax*h), vay=_mm256_set1_ps(params.ay*h), vaz=_mm256_set1_ps(params.az*h);
    for (int s=0;s<substeps;++s){
        size_t n=cloth.particles.size(); size_t i=0; for (; i+8<=n; i+=8){
            float invm[8]; for(int k=0;k<8;++k) invm[k]=cloth.particles[i+k].inv_mass;
            __m256 mInv = _mm256_loadu_ps(invm);
            __m256 zero=_mm256_set1_ps(0.0f);
            // load vx,vy,vz
            float vx[8],vy[8],vz[8],x[8],y[8],z[8]; for(int k=0;k<8;++k){ auto& p=cloth.particles[i+k]; vx[k]=p.vx; vy[k]=p.vy; vz[k]=p.vz; x[k]=p.x; y[k]=p.y; z[k]=p.z; p.corr_x=p.corr_y=p.corr_z=0.0f; }
            __m256 mvx=_mm256_loadu_ps(vx), mvy=_mm256_loadu_ps(vy), mvz=_mm256_loadu_ps(vz);
            // mask for movable
            __m256 mask=_mm256_cmp_ps(mInv, zero, _CMP_NEQ_OQ);
            mvx=_mm256_blendv_ps(mvx, _mm256_add_ps(mvx,vax), mask);
            mvy=_mm256_blendv_ps(mvy, _mm256_add_ps(mvy,vay), mask);
            mvz=_mm256_blendv_ps(mvz, _mm256_add_ps(mvz,vaz), mask);
            __m256 mx=_mm256_loadu_ps(x), my=_mm256_loadu_ps(y), mz=_mm256_loadu_ps(z);
            // store px,py,pz and advance x,y,z
            for(int k=0;k<8;++k){ auto& p=cloth.particles[i+k]; p.px=p.x; p.py=p.y; p.pz=p.z; }
            mx=_mm256_add_ps(mx, _mm256_mul_ps(mvx,_mm256_set1_ps(h)));
            my=_mm256_add_ps(my, _mm256_mul_ps(mvy,_mm256_set1_ps(h)));
            mz=_mm256_add_ps(mz, _mm256_mul_ps(mvz,_mm256_set1_ps(h)));
            _mm256_storeu_ps(vx, mvx); _mm256_storeu_ps(vy, mvy); _mm256_storeu_ps(vz, mvz);
            _mm256_storeu_ps(x, mx); _mm256_storeu_ps(y, my); _mm256_storeu_ps(z, mz);
            for(int k=0;k<8;++k){ auto& p=cloth.particles[i+k]; if (p.inv_mass!=0.0f){ p.vx=vx[k]; p.vy=vy[k]; p.vz=vz[k]; p.x=x[k]; p.y=y[k]; p.z=z[k]; } }
        }
        for (; i<n; ++i){ auto& p=cloth.particles[i]; p.corr_x=p.corr_y=p.corr_z=0.0f; if (p.inv_mass==0.0f){ p.vx=p.vy=p.vz=0; p.px=p.x; p.py=p.y; p.pz=p.z; } else { p.vx+=params.ax*h; p.vy+=params.ay*h; p.vz+=params.az*h; p.px=p.x; p.py=p.y; p.pz=p.z; p.x+=p.vx*h; p.y+=p.vy*h; p.z+=p.vz*h; } }
        // native constraints
        const float alpha_dt=1.0f/(h*h);
        for (int it=0; it<std::max(1, params.iterations); ++it) {
            for (auto& c : cloth.constraints) {
                auto& pi=cloth.particles[c.i]; auto& pj=cloth.particles[c.j]; float dx=pi.x-pj.x, dy=pi.y-pj.y, dz=pi.z-pj.z; float dist=std::sqrt(dx*dx+dy*dy+dz*dz); if (dist<1e-8f){ if(params.write_debug_fields){ c.last_c=0;c.last_dlambda=0;c.last_nx=c.last_ny=c.last_nz=0;} continue; }
                float C=dist-c.rest_length; float nx=dx/dist, ny=dy/dist, nz=dz/dist; float scale=params.compliance_scale_all; switch(c.type){case ConstraintType::Structural: scale*=params.compliance_scale_structural; break; case ConstraintType::Shear: scale*=params.compliance_scale_shear; break; case ConstraintType::Bending: scale*=params.compliance_scale_bending; break; default: break;}
                float alpha_tilde=(c.compliance*scale)*alpha_dt; float wsum=pi.inv_mass+pj.inv_mass; float denom=wsum+alpha_tilde; if (denom<=0.0f){ if(params.write_debug_fields){ c.last_c=C; c.last_dlambda=0; c.last_nx=nx;c.last_ny=ny;c.last_nz=nz;} continue; }
                float dl=(-C - alpha_tilde*c.lambda)/denom; c.lambda+=dl; float sx=dl*nx, sy=dl*ny, sz=dl*nz; if (params.max_correction>0.0f){ float mag=std::sqrt(sx*sx+sy*sy+sz*sz); if (mag>params.max_correction&&mag>0.0f){ float r=params.max_correction/mag; sx*=r; sy*=r; sz*=r; }} if (pi.inv_mass>0){ pi.x+=pi.inv_mass*sx; pi.y+=pi.inv_mass*sy; pi.z+=pi.inv_mass*sz; } if (pj.inv_mass>0){ pj.x-=pj.inv_mass*sx; pj.y-=pj.inv_mass*sy; pj.z-=pj.inv_mass*sz; }
                if (params.write_debug_fields){ c.last_c=C; c.last_dlambda=dl; c.last_nx=nx;c.last_ny=ny;c.last_nz=nz; }
            }
        }
        // velocities
        float inv_h=1.0f/h; i=0; for(; i+8<=n; i+=8){ float px[8],py[8],pz[8],x[8],y[8],z[8]; for(int k=0;k<8;++k){ auto& p=cloth.particles[i+k]; px[k]=p.px; py[k]=p.py; pz[k]=p.pz; x[k]=p.x; y[k]=p.y; z[k]=p.z; }
            __m256 mx=_mm256_loadu_ps(x), my=_mm256_loadu_ps(y), mz=_mm256_loadu_ps(z); __m256 mpx=_mm256_loadu_ps(px), mpy=_mm256_loadu_ps(py), mpz=_mm256_loadu_ps(pz); __m256 invh=_mm256_set1_ps(inv_h);
            __m256 mvx=_mm256_mul_ps(_mm256_sub_ps(mx,mpx),invh); __m256 mvy=_mm256_mul_ps(_mm256_sub_ps(my,mpy),invh); __m256 mvz=_mm256_mul_ps(_mm256_sub_ps(mz,mpz),invh);
            _mm256_storeu_ps(x, mvx); _mm256_storeu_ps(y, mvy); _mm256_storeu_ps(z, mvz); for(int k=0;k<8;++k){ auto& p=cloth.particles[i+k]; p.vx=x[k]; p.vy=y[k]; p.vz=z[k]; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); p.vx*=s; p.vy*=s; p.vz*=s; } }
        }
        for(; i<n; ++i){ auto& p=cloth.particles[i]; float invh=1.0f/h; p.vx=(p.x-p.px)*invh; p.vy=(p.y-p.py)*invh; p.vz=(p.z-p.pz)*invh; if (params.velocity_damping>0.0f){ float s=std::max(0.0f,1.0f-params.velocity_damping); p.vx*=s; p.vy*=s; p.vz*=s; } }
    }
    cloth.last_dt=clamped_dt; cloth.last_iterations=params.iterations;
#else
    xpbd_step_native_aos(cloth, dt, params);
#endif
}

} // namespace HinaPE

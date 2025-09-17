#include "aos/solver_xpbd_aos.h"
#include <algorithm>
#include <cmath>

namespace HinaPE {

void xpbd_step_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
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

} // namespace HinaPE

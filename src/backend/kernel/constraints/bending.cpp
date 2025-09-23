#include "bending.h"
#include "backend/storage/soa.h"
#include "backend/storage/aosoa.h"
#include "backend/storage/aos.h"
#include "core/common/utils.h"
#include <cmath>
#include <algorithm>

namespace sim {
    static float dihedral(SoAView3& pos, unsigned int i0, unsigned int i1, unsigned int i2, unsigned int i3,
                          float& n1x, float& n1y, float& n1z, float& n2x, float& n2y, float& n2z) {
        float e0x = pos.x[i1]-pos.x[i0], e0y = pos.y[i1]-pos.y[i0], e0z = pos.z[i1]-pos.z[i0];
        float e1x = pos.x[i2]-pos.x[i0], e1y = pos.y[i2]-pos.y[i0], e1z = pos.z[i2]-pos.z[i0];
        float e2x = pos.x[i3]-pos.x[i0], e2y = pos.y[i3]-pos.y[i0], e2z = pos.z[i3]-pos.z[i0];
        util::cross3(e0x,e0y,e0z, e1x,e1y,e1z, n1x,n1y,n1z);
        util::cross3(e0x,e0y,e0z, e2x,e2y,e2z, n2x,n2y,n2z);
        float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
        if (n1l <= 1e-12f || n2l <= 1e-12f) return 0.0f;
        float c = util::dot3(n1x,n1y,n1z, n2x,n2y,n2z) / (n1l*n2l);
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }

    void kernel_bending_project(const unsigned int* quads, size_t m, SoAView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt) {
        (void)alpha; (void)dt;
        if (!quads || !target || m == 0) return;
        const float k = 0.1f;
        for (int it = 0; it < iterations; ++it) {
            for (size_t qi = 0; qi < m; ++qi) {
                unsigned int i0 = quads[4*qi+0];
                unsigned int i1 = quads[4*qi+1];
                unsigned int i2 = quads[4*qi+2];
                unsigned int i3 = quads[4*qi+3];
                if (i0>=pos.n || i1>=pos.n || i2>=pos.n || i3>=pos.n) continue;
                float n1x,n1y,n1z,n2x,n2y,n2z;
                float theta = dihedral(pos, i0,i1,i2,i3, n1x,n1y,n1z, n2x,n2y,n2z);
                float err = theta - target[qi];
                if (std::fabs(err) < 1e-6f) continue;
                float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
                if (n1l > 1e-12f) { n1x/=n1l; n1y/=n1l; n1z/=n1l; }
                if (n2l > 1e-12f) { n2x/=n2l; n2y/=n2l; n2z/=n2l; }
                float s = -k * err;
                float w2 = inv_mass ? inv_mass[i2] : 1.0f;
                float w3 = inv_mass ? inv_mass[i3] : 1.0f;
                if (w2 > 0.0f) { pos.x[i2] += s * w2 * n1x; pos.y[i2] += s * w2 * n1y; pos.z[i2] += s * w2 * n1z; }
                if (w3 > 0.0f) { pos.x[i3] -= s * w3 * n2x; pos.y[i3] -= s * w3 * n2y; pos.z[i3] -= s * w3 * n2z; }
            }
        }
    }

    static float dihedral_aosoa(AoSoAView3& pos, unsigned int i0, unsigned int i1, unsigned int i2, unsigned int i3,
                          float& n1x, float& n1y, float& n1z, float& n2x, float& n2y, float& n2z) {
        float p0x,p0y,p0z,p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
        storage_aosoa_read3(pos,i0,p0x,p0y,p0z);
        storage_aosoa_read3(pos,i1,p1x,p1y,p1z);
        storage_aosoa_read3(pos,i2,p2x,p2y,p2z);
        storage_aosoa_read3(pos,i3,p3x,p3y,p3z);
        float e0x = p1x-p0x, e0y = p1y-p0y, e0z = p1z-p0z;
        float e1x = p2x-p0x, e1y = p2y-p0y, e1z = p2z-p0z;
        float e2x = p3x-p0x, e2y = p3y-p0y, e2z = p3z-p0z;
        util::cross3(e0x,e0y,e0z, e1x,e1y,e1z, n1x,n1y,n1z);
        util::cross3(e0x,e0y,e0z, e2x,e2y,e2z, n2x,n2y,n2z);
        float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
        if (n1l <= 1e-12f || n2l <= 1e-12f) return 0.0f;
        float c = util::dot3(n1x,n1y,n1z, n2x,n2y,n2z) / (n1l*n2l);
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }

    void kernel_bending_project_aosoa(const unsigned int* quads, size_t m, AoSoAView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt) {
        (void)alpha; (void)dt;
        if (!quads || !target || m == 0) return;
        const float k = 0.1f;
        for (int it = 0; it < iterations; ++it) {
            for (size_t qi = 0; qi < m; ++qi) {
                unsigned int i0 = quads[4*qi+0];
                unsigned int i1 = quads[4*qi+1];
                unsigned int i2 = quads[4*qi+2];
                unsigned int i3 = quads[4*qi+3];
                if (i0>=pos.n || i1>=pos.n || i2>=pos.n || i3>=pos.n) continue;
                float n1x,n1y,n1z,n2x,n2y,n2z;
                float theta = dihedral_aosoa(pos, i0,i1,i2,i3, n1x,n1y,n1z, n2x,n2y,n2z);
                float err = theta - target[qi];
                if (std::fabs(err) < 1e-6f) continue;
                float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
                if (n1l > 1e-12f) { n1x/=n1l; n1y/=n1l; n1z/=n1l; }
                if (n2l > 1e-12f) { n2x/=n2l; n2y/=n2l; n2z/=n2l; }
                float s = -k * err;
                float w2 = inv_mass ? inv_mass[i2] : 1.0f;
                float w3 = inv_mass ? inv_mass[i3] : 1.0f;
                if (w2 > 0.0f) storage_aosoa_axpy3(pos, i2, s * w2 * n1x, s * w2 * n1y, s * w2 * n1z);
                if (w3 > 0.0f) storage_aosoa_axpy3(pos, i3, -s * w3 * n2x, -s * w3 * n2y, -s * w3 * n2z);
            }
        }
    }

    static float dihedral_aos(AoSView3& pos, unsigned int i0, unsigned int i1, unsigned int i2, unsigned int i3,
                          float& n1x, float& n1y, float& n1z, float& n2x, float& n2y, float& n2z) {
        float p0x,p0y,p0z,p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
        storage_aos_read3(pos,i0,p0x,p0y,p0z);
        storage_aos_read3(pos,i1,p1x,p1y,p1z);
        storage_aos_read3(pos,i2,p2x,p2y,p2z);
        storage_aos_read3(pos,i3,p3x,p3y,p3z);
        float e0x = p1x-p0x, e0y = p1y-p0y, e0z = p1z-p0z;
        float e1x = p2x-p0x, e1y = p2y-p0y, e1z = p2z-p0z;
        float e2x = p3x-p0x, e2y = p3y-p0y, e2z = p3z-p0z;
        util::cross3(e0x,e0y,e0z, e1x,e1y,e1z, n1x,n1y,n1z);
        util::cross3(e0x,e0y,e0z, e2x,e2y,e2z, n2x,n2y,n2z);
        float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
        if (n1l <= 1e-12f || n2l <= 1e-12f) return 0.0f;
        float c = util::dot3(n1x,n1y,n1z, n2x,n2y,n2z) / (n1l*n2l);
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }

    void kernel_bending_project_aos(const unsigned int* quads, size_t m, AoSView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt) {
        (void)alpha; (void)dt;
        if (!quads || !target || m == 0) return;
        const float k = 0.1f;
        for (int it = 0; it < iterations; ++it) {
            for (size_t qi = 0; qi < m; ++qi) {
                unsigned int i0 = quads[4*qi+0];
                unsigned int i1 = quads[4*qi+1];
                unsigned int i2 = quads[4*qi+2];
                unsigned int i3 = quads[4*qi+3];
                if (i0>=pos.n || i1>=pos.n || i2>=pos.n || i3>=pos.n) continue;
                float n1x,n1y,n1z,n2x,n2y,n2z;
                float theta = dihedral_aos(pos, i0,i1,i2,i3, n1x,n1y,n1z, n2x,n2y,n2z);
                float err = theta - target[qi];
                if (std::fabs(err) < 1e-6f) continue;
                float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
                if (n1l > 1e-12f) { n1x/=n1l; n1y/=n1l; n1z/=n1l; }
                if (n2l > 1e-12f) { n2x/=n2l; n2y/=n2l; n2z/=n2l; }
                float s = -k * err;
                float w2 = inv_mass ? inv_mass[i2] : 1.0f;
                float w3 = inv_mass ? inv_mass[i3] : 1.0f;
                if (w2 > 0.0f) storage_aos_axpy3(pos, i2, s * w2 * n1x, s * w2 * n1y, s * w2 * n1z);
                if (w3 > 0.0f) storage_aos_axpy3(pos, i3, -s * w3 * n2x, -s * w3 * n2y, -s * w3 * n2z);
            }
        }
    }
}

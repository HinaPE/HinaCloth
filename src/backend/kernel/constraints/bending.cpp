#include "bending.h"
#include "backend/storage/soa.h"
#include <cmath>
#include <algorithm>

namespace sim {
    static inline void cross3(float ax, float ay, float az, float bx, float by, float bz, float& rx, float& ry, float& rz){ rx = ay*bz - az*by; ry = az*bx - ax*bz; rz = ax*by - ay*bx; }
    static inline float dot3(float ax, float ay, float az, float bx, float by, float bz){ return ax*bx+ay*by+az*bz; }
    static inline float len3(float x, float y, float z){ return std::sqrt(x*x+y*y+z*z); }

    static float dihedral(SoAView3& pos, unsigned int i0, unsigned int i1, unsigned int i2, unsigned int i3,
                          float& n1x, float& n1y, float& n1z, float& n2x, float& n2y, float& n2z) {
        float e0x = pos.x[i1]-pos.x[i0], e0y = pos.y[i1]-pos.y[i0], e0z = pos.z[i1]-pos.z[i0];
        float e1x = pos.x[i2]-pos.x[i0], e1y = pos.y[i2]-pos.y[i0], e1z = pos.z[i2]-pos.z[i0];
        float e2x = pos.x[i3]-pos.x[i0], e2y = pos.y[i3]-pos.y[i0], e2z = pos.z[i3]-pos.z[i0];
        cross3(e0x,e0y,e0z, e1x,e1y,e1z, n1x,n1y,n1z);
        cross3(e0x,e0y,e0z, e2x,e2y,e2z, n2x,n2y,n2z);
        float n1l = len3(n1x,n1y,n1z); float n2l = len3(n2x,n2y,n2z);
        if (n1l <= 1e-12f || n2l <= 1e-12f) return 0.0f;
        float c = dot3(n1x,n1y,n1z, n2x,n2y,n2z) / (n1l*n2l);
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }

    void kernel_bending_project(const unsigned int* quads, size_t m, SoAView3& pos, const float* target, int iterations, float /*alpha*/, float /*dt*/) {
        if (!quads || !target || m == 0) return;
        const float k = 0.1f; // simple stiffness per iteration
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
                // Move i2 against its face normal, i3 along its face normal
                float n1l = len3(n1x,n1y,n1z); float n2l = len3(n2x,n2y,n2z);
                if (n1l > 1e-12f) { n1x/=n1l; n1y/=n1l; n1z/=n1l; }
                if (n2l > 1e-12f) { n2x/=n2l; n2y/=n2l; n2z/=n2l; }
                float s = -k * err;
                pos.x[i2] += s * n1x; pos.y[i2] += s * n1y; pos.z[i2] += s * n1z;
                pos.x[i3] -= s * n2x; pos.y[i3] -= s * n2y; pos.z[i3] -= s * n2z;
            }
        }
    }
}

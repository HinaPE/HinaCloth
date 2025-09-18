#ifndef HINACLOTH_INCLUDE_AOS_CLOTH_DATA_AOS_H
#define HINACLOTH_INCLUDE_AOS_CLOTH_DATA_AOS_H

// AoS cloth data (Array of Structs)

#include "cloth_types.h"

#include <vector>

namespace HinaPE {

struct ParticleAOS {
    float x{0.0f}, y{0.0f}, z{0.0f};
    float px{0.0f}, py{0.0f}, pz{0.0f};
    float vx{0.0f}, vy{0.0f}, vz{0.0f};
    float inv_mass{1.0f};
    float corr_x{0.0f}, corr_y{0.0f}, corr_z{0.0f};
};

struct DistanceConstraintAOS {
    int i{0};
    int j{0};
    float rest_length{0.0f};
    float compliance{0.0f};
    float lambda{0.0f};
    ConstraintType type{ConstraintType::Structural};
    float last_c{0.0f};
    float last_dlambda{0.0f};
    float last_nx{0.0f}, last_ny{0.0f}, last_nz{0.0f};
};

struct ClothAOS {
    int nx{0};
    int ny{0};
    std::vector<ParticleAOS> particles;
    std::vector<DistanceConstraintAOS> constraints;
    float last_dt{0.0f};
    int last_iterations{0};
};

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_AOS_CLOTH_DATA_AOS_H

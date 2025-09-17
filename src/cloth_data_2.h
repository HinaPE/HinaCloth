#ifndef HINACLOTH_CLOTH_DATA_2_H
#define HINACLOTH_CLOTH_DATA_2_H

#include <cstdint>
#include <vector>

namespace HinaPE {

using u8  = std::uint8_t;
using u32 = std::uint32_t;

// A very simple SoA cloth container without any allocator/stride tricks.
// All arrays are std::vector with contiguous storage.
struct ClothData2 {
    // Particles
    std::vector<float> px, py, pz;   // positions
    std::vector<float> vx, vy, vz;   // velocities
    std::vector<float> inv_mass;     // inverse mass (0 for static)
    std::vector<u8>    pinned;       // 1 if pinned, 0 otherwise

    // Distance constraints (edges)
    std::vector<u32>   edge_i, edge_j;
    std::vector<float> rest;         // rest length
    std::vector<float> compliance;   // compliance value per edge
    std::vector<float> lambda;       // XPBD lambda (persistent for soft constraints)
    std::vector<float> alpha;        // computed alpha_tilde (debug/inspection)
    std::vector<u8>    color;        // color (bucket) per edge (optional)

    // Allocation helpers
    void allocateParticles(std::size_t n) {
        px.assign(n, 0.0f); py.assign(n, 0.0f); pz.assign(n, 0.0f);
        vx.assign(n, 0.0f); vy.assign(n, 0.0f); vz.assign(n, 0.0f);
        inv_mass.assign(n, 0.0f);
        pinned.assign(n, 0);
    }

    void allocateDistance(std::size_t m) {
        edge_i.assign(m, 0u); edge_j.assign(m, 0u);
        rest.assign(m, 0.0f);
        compliance.assign(m, 0.0f);
        lambda.assign(m, 0.0f);
        alpha.assign(m, 0.0f);
        color.assign(m, 0u);
    }

    // Counts
    std::size_t numParticles() const { return px.size(); }
    std::size_t numEdges() const { return edge_i.size(); }
};

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_2_H


#ifndef HINACLOTH_DATA_H
#define HINACLOTH_DATA_H
#include <vector>

namespace sim {
    struct Model;
    struct BuildDesc;
    struct Command;
    struct RemapPlan;

    struct Data {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> z;
        std::vector<float> vx;
        std::vector<float> vy;
        std::vector<float> vz;
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
        // XPBD node properties
        std::vector<float> inv_mass;   // per node inverse mass (0 -> pinned)
        // XPBD constraint state
        std::vector<float> lambda_edge; // per-edge lambda accumulator
        // Global parameters
        float gx;
        float gy;
        float gz;
        float distance_compliance; // compliance for distance constraints (0 => PBD)
        // Solve parameters (policy defaults, overridable at runtime)
        int   solve_substeps;
        int   solve_iterations;
        float solve_damping;
        // Exec parameters (from Policy.exec)
        bool  exec_use_tbb;
        int   exec_threads; // -1 auto
        bool  exec_use_avx2;
        // Layout selection (Stage 3)
        bool  exec_layout_blocked; // true => use AoSoA blocked layout during solve
        unsigned int layout_block_size; // AoSoA block size (8/16)
        std::vector<float> pos_aosoa;   // 3*block*ceil(n/block) packed positions buffer for AoSoA
    };

    bool core_data_create_from_state(const BuildDesc& in, const Model& m, Data*& out);
    bool core_data_apply_overrides(Data& d, const Command* cmds, size_t count);
    bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd);
    void core_data_destroy(Data* d);
}

#endif //HINACLOTH_DATA_H

/*
 * File: data.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_DATA_H
#define HINACLOTH_DATA_H
#include <vector>
#include "core/common/types.h"

namespace sim {
    struct Model;
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
        std::vector<float> inv_mass;
        std::vector<float> lambda_edge;
        float gx{0.0f};
        float gy{-9.8f};
        float gz{0.0f};
        float distance_compliance{0.0f};
        std::vector<float> distance_compliance_edge;
        std::vector<float> distance_alpha_edge;
        int   solve_substeps{1};
        int   solve_iterations{8};
        float solve_damping{0.0f};
        bool  exec_use_tbb{false};
        int   exec_threads{-1};
        bool  exec_use_avx2{false};
        bool  exec_layout_blocked{false};
        unsigned int layout_block_size{8u};
        std::vector<float> pos_aosoa;
        bool  op_enable_attachment{false};
        bool  op_enable_bending{false};
        std::vector<float> attach_w;
        std::vector<float> attach_tx;
        std::vector<float> attach_ty;
        std::vector<float> attach_tz;
    };

    [[nodiscard]] bool core_data_create_from_state(const eng::BuildDesc& in, const Model& m, Data*& out);
    [[nodiscard]] bool core_data_apply_overrides(Data& d, const eng::Command* cmds, size_t count);
    [[nodiscard]] bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd);
    void core_data_destroy(Data* d) noexcept;
}

#endif

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
        float gx;
        float gy;
        float gz;
        float distance_compliance;
        std::vector<float> distance_compliance_edge;
        std::vector<float> distance_alpha_edge;
        int   solve_substeps;
        int   solve_iterations;
        float solve_damping;
        bool  exec_use_tbb;
        int   exec_threads;
        bool  exec_use_avx2;
        bool  exec_layout_blocked;
        unsigned int layout_block_size;
        std::vector<float> pos_aosoa;
        bool  op_enable_attachment;
        bool  op_enable_bending;
        std::vector<float> attach_w;
        std::vector<float> attach_tx;
        std::vector<float> attach_ty;
        std::vector<float> attach_tz;
    };

    bool core_data_create_from_state(const eng::BuildDesc& in, const Model& m, Data*& out);
    bool core_data_apply_overrides(Data& d, const eng::Command* cmds, size_t count);
    bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd);
    void core_data_destroy(Data* d);
}

#endif

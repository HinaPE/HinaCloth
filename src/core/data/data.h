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
        float gx;
        float gy;
        float gz;
    };

    bool core_data_create_from_state(const BuildDesc& in, const Model& m, Data*& out);
    bool core_data_apply_overrides(Data& d, const Command* cmds, size_t count);
    bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd);
    void core_data_destroy(Data* d);
}

#endif //HINACLOTH_DATA_H

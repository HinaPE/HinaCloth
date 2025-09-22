/*
 * File: model.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_MODEL_H
#define HINACLOTH_MODEL_H
#include <cstdint>
#include <vector>

namespace sim {
    struct Model {
        uint32_t node_count;
        std::vector<uint32_t> edges;
        std::vector<float> rest;
        uint32_t island_count{1};
        std::vector<uint32_t> island_offsets;
        std::vector<uint32_t> node_remap;
        uint32_t layout_block_size{8};
        std::vector<uint32_t> bend_pairs;
        std::vector<float>    bend_rest_angle;
    };

    void core_model_destroy(Model* m);
}
#endif

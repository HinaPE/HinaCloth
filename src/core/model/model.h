#ifndef HINACLOTH_MODEL_H
#define HINACLOTH_MODEL_H
#include <cstdint>
#include <vector>

namespace sim {
    struct Model {
        uint32_t node_count;
        std::vector<uint32_t> edges; // pairs, reordered by island
        std::vector<float> rest;     // reordered by island
        // Islanding metadata
        uint32_t island_count{1};
        std::vector<uint32_t> island_offsets; // size = island_count+1, edge counts prefix sum
        // Layout plan (Stage 3)
        std::vector<uint32_t> node_remap; // author index -> internal index
        uint32_t layout_block_size{8};    // preferred AoSoA block size
    };

    void core_model_destroy(Model* m);
}
#endif //HINACLOTH_MODEL_H

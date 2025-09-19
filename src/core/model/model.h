#ifndef HINACLOTH_MODEL_H
#define HINACLOTH_MODEL_H
#include <cstdint>
#include <vector>

namespace sim {
    struct Model {
        uint32_t node_count;
        std::vector<uint32_t> edges;
        std::vector<float> rest;
    };

    void core_model_destroy(Model* m);
}
#endif //HINACLOTH_MODEL_H

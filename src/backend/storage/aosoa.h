#ifndef HINACLOTH_AOSOA_H
#define HINACLOTH_AOSOA_H
#include <cstdint>
namespace sim {
    struct AoSoAView3 {
        float* base;
        size_t n;
        size_t block;
        size_t stride;
    };
    inline void storage_bind_aosoa(AoSoAView3& v, float* base, size_t n, size_t block, size_t stride) {
        v.base=base; v.n=n; v.block=block; v.stride=stride;
    }
}
#endif //HINACLOTH_AOSOA_H
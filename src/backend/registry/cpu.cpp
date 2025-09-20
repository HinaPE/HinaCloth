#include "cpu.h"
#if defined(_MSC_VER)
  #include <intrin.h>
#else
  #include <cpuid.h>
#endif

namespace sim {
    bool cpu_has_avx2() {
        // Check CPUID leaf 7, EBX bit 5 (AVX2)
        int regs[4] = {0,0,0,0};
    #if defined(_MSC_VER)
        __cpuid(regs, 0);
        int max_leaf = regs[0];
        if (max_leaf >= 7) {
            int regs7[4];
            __cpuidex(regs7, 7, 0);
            return (regs7[1] & (1 << 5)) != 0; // EBX bit 5
        }
        return false;
    #else
        unsigned int eax=0, ebx=0, ecx=0, edx=0;
        __cpuid(0, eax, ebx, ecx, edx);
        if (eax >= 7) {
            unsigned int eax7, ebx7, ecx7, edx7;
            __cpuid_count(7, 0, eax7, ebx7, ecx7, edx7);
            return (ebx7 & (1u << 5)) != 0;
        }
        return false;
    #endif
    }
}


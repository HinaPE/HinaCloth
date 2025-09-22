#include "cpu.h"
#if defined(_MSC_VER)
  #if defined(_M_X64) || defined(_M_IX86)
    #include <intrin.h>
  #endif
#else
  #if defined(__x86_64__) || defined(__i386__)
    #include <cpuid.h>
  #endif
#endif

namespace sim {
    bool cpu_has_avx2() {
    #if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
        int regs[4] = {0,0,0,0};
        __cpuid(regs, 0);
        int max_leaf = regs[0];
        if (max_leaf >= 7) {
            int regs7[4] = {0,0,0,0};
            __cpuidex(regs7, 7, 0);
            return (regs7[1] & (1 << 5)) != 0;
        }
        return false;
    #elif (defined(__x86_64__) || defined(__i386__)) && (defined(__GNUC__) || defined(__clang__))
        unsigned int eax=0, ebx=0, ecx=0, edx=0;
        __cpuid(0, eax, ebx, ecx, edx);
        if (eax >= 7) {
            unsigned int eax7=0, ebx7=0, ecx7=0, edx7=0;
            __cpuid_count(7, 0, eax7, ebx7, ecx7, edx7);
            return (ebx7 & (1u << 5)) != 0;
        }
        return false;
    #else
        return false;
    #endif
    }
}

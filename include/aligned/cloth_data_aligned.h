#ifndef HINACLOTH_INCLUDE_ALIGNED_CLOTH_DATA_ALIGNED_H
#define HINACLOTH_INCLUDE_ALIGNED_CLOTH_DATA_ALIGNED_H

// High-performance aligned SoA cloth data for XPBD

#include "cloth_types.h"

#include <cstddef>
#include <cstdlib>
#if defined(_MSC_VER)
#include <malloc.h>
#endif
#include <memory>
#include <new>
#include <type_traits>
#include <vector>

namespace HinaPE {

template <typename T, std::size_t Alignment>
class AlignedAllocator {
public:
    using value_type = T;
    using propagate_on_container_move_assignment = std::true_type;
    using is_always_equal = std::true_type;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    template <typename U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment>;
    };

    constexpr AlignedAllocator() noexcept = default;

    template <typename U>
    constexpr explicit AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    [[nodiscard]] T* allocate(std::size_t n) {
        if (n == 0) {
            return nullptr;
        }
        const auto requested = n * sizeof(T);
        const auto bytes = requested == 0 ? Alignment : ((requested + Alignment - 1) / Alignment) * Alignment;
#if defined(_MSC_VER)
        if (void* ptr = ::_aligned_malloc(bytes, Alignment)) {
            return static_cast<T*>(ptr);
        }
        throw std::bad_alloc{};
#else
        void* ptr = nullptr;
        if (const int rc = ::posix_memalign(&ptr, Alignment, bytes); rc == 0) {
            return static_cast<T*>(ptr);
        }
        throw std::bad_alloc{};
#endif
    }

    void deallocate(T* ptr, std::size_t) noexcept {
#if defined(_MSC_VER)
        ::_aligned_free(ptr);
#else
        std::free(ptr);
#endif
    }

    template <typename U>
    [[nodiscard]] constexpr bool operator==(const AlignedAllocator<U, Alignment>&) const noexcept {
        return true;
    }

    template <typename U>
    [[nodiscard]] constexpr bool operator!=(const AlignedAllocator<U, Alignment>&) const noexcept {
        return false;
    }
};

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, 64>>;

struct ClothAligned {
    int nx{0};
    int ny{0};
    AlignedVector<float> x, y, z;
    AlignedVector<float> px, py, pz;
    AlignedVector<float> vx, vy, vz;
    AlignedVector<float> inv_mass;
    AlignedVector<float> corr_x, corr_y, corr_z;
    AlignedVector<int> ci, cj;
    AlignedVector<float> rest_length, compliance, lambda;
    AlignedVector<ConstraintType> type;
    AlignedVector<float> last_c, last_dlambda, last_nx, last_ny, last_nz;
    float last_dt{0.0f};
    int last_iterations{0};
};

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_ALIGNED_CLOTH_DATA_ALIGNED_H

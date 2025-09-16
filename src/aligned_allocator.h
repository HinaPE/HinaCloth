#ifndef HINACLOTH_ALIGNED_ALLOCATOR_H
#define HINACLOTH_ALIGNED_ALLOCATOR_H

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mdspan>
#include <memory_resource>
#include <new>
#include <span>
#include <typeindex>
#include <vector>
#if defined(_MSC_VER)
#include <bit>
#endif

namespace HinaPE {
    class AlignedResource final : public std::pmr::memory_resource {
    public:
        explicit AlignedResource(std::size_t alignment = 64, std::pmr::memory_resource* upstream = std::pmr::get_default_resource()) : align_(alignment), upstream_(upstream) {}

    private:
        std::size_t align_;
        std::pmr::memory_resource* upstream_;
        void* do_allocate(std::size_t bytes, std::size_t alignment) override {
            const std::size_t A = align_ > alignment ? align_ : alignment;
            assert(A >= alignof(void*));
            assert(std::has_single_bit(A));
#if defined(_MSC_VER)
            void* p = _aligned_malloc(bytes, A);
            if (!p) {
                throw std::bad_alloc{};
            }
            return p;
#else
            void* p = nullptr;
            if (posix_memalign(&p, A, bytes) != 0) {
                throw std::bad_alloc{};
            }
            return p;
#endif
        }
        void do_deallocate(void* p, std::size_t, std::size_t) override {
#if defined(_MSC_VER)
            _aligned_free(p);
#else
            std::free(p);
#endif
        }
        [[nodiscard]] bool do_is_equal(const memory_resource& other) const noexcept override {
            return this == &other;
        }
    };

    struct ColumnDesc {
        void* data{nullptr};
        size_t bytes{0};
        size_t count{0};
        size_t stride_bytes{0};
        size_t alignment{64};
        bool owns{true};
        size_t elem_size{0};
        std::type_index type{typeid(void)};
    };

    using ColumnId = std::uint32_t;
    struct no_init_t {};
    inline constexpr no_init_t no_init{};

    class ColumnArena {
    public:
        explicit ColumnArena(std::pmr::memory_resource* mr) : mr_(mr) {}
        ~ColumnArena() {
            release_all();
        }
        ColumnArena(const ColumnArena&)            = delete;
        ColumnArena& operator=(const ColumnArena&) = delete;
        ColumnArena(ColumnArena&&)                 = delete;
        ColumnArena& operator=(ColumnArena&&)      = delete;

        template <class T>
        [[nodiscard]] ColumnId make_column(size_t count, size_t stride_bytes = sizeof(T), size_t alignment = 64) {
            check_params<T>(count, stride_bytes, alignment);
            const size_t bytes = stride_bytes * count;
            void* p            = mr_->allocate(bytes, alignment);
            std::memset(p, 0, bytes);
            return emplace_desc<T>(p, bytes, count, stride_bytes, alignment, true);
        }
        template <class T>
        [[nodiscard]] ColumnId make_column(size_t count, no_init_t, size_t stride_bytes = sizeof(T), size_t alignment = 64) {
            check_params<T>(count, stride_bytes, alignment);
            const size_t bytes = stride_bytes * count;
            void* p            = mr_->allocate(bytes, alignment);
            return emplace_desc<T>(p, bytes, count, stride_bytes, alignment, true);
        }
        template <class T>
        [[nodiscard]] ColumnId map_external(T* ptr, size_t count, size_t stride_bytes = sizeof(T), size_t alignment = alignof(T)) {
            check_params<T>(count, stride_bytes, alignment);
            return emplace_desc<T>(ptr, stride_bytes * count, count, stride_bytes, alignment, false);
        }
        template <class T>
        [[nodiscard]] auto bind(ColumnId id) {
            auto& c = cols_.at(id);
            debug_check_type<T>(c);
            debug_check_contiguous<T>(c);
            return std::mdspan<T, std::extents<size_t, std::dynamic_extent>>(static_cast<T*>(c.data), c.count);
        }
        template <class T>
        [[nodiscard]] auto bind_const(ColumnId id) const {
            const auto& c = cols_.at(id);
            debug_check_type<T>(c);
            debug_check_contiguous<T>(c);
            return std::mdspan<const T, std::extents<size_t, std::dynamic_extent>>(static_cast<const T*>(c.data), c.count);
        }
        template <class T>
        [[nodiscard]] auto bind_strided(ColumnId id) {
            auto& c = cols_.at(id);
            debug_check_type<T>(c);
            using ext    = std::extents<size_t, std::dynamic_extent>;
            using layout = std::layout_stride;
            return std::mdspan<T, ext, layout>(static_cast<T*>(c.data), ext(c.count), std::array{c.stride_bytes / sizeof(T)});
        }
        template <class T>
        [[nodiscard]] auto bind_strided_const(ColumnId id) const {
            const auto& c = cols_.at(id);
            debug_check_type<T>(c);
            using ext    = std::extents<size_t, std::dynamic_extent>;
            using layout = std::layout_stride;
            return std::mdspan<const T, ext, layout>(static_cast<const T*>(c.data), ext(c.count), std::array{c.stride_bytes / sizeof(T)});
        }
        template <class T>
        void reallocate_preserve(ColumnId id, size_t new_count, size_t new_stride_bytes, size_t new_alignment) {
            auto& c = cols_.at(id);
            debug_check_type<T>(c);
            check_params<T>(new_count, new_stride_bytes, new_alignment);
            void* p                 = mr_->allocate(new_stride_bytes * new_count, new_alignment);
            const size_t copy_elems = c.count < new_count ? c.count : new_count;
            if (const size_t elem = sizeof(T); c.stride_bytes == elem && new_stride_bytes == elem) {
                std::memcpy(p, c.data, copy_elems * elem);
            } else {
                auto* src = static_cast<const std::byte*>(c.data);
                auto* dst = static_cast<std::byte*>(p);
                for (size_t i = 0; i < copy_elems; ++i) {
                    std::memcpy(dst + i * new_stride_bytes, src + i * c.stride_bytes, elem);
                }
            }
            if (c.owns && c.data) {
                mr_->deallocate(c.data, c.bytes, c.alignment);
            }
            c.data         = p;
            c.bytes        = new_stride_bytes * new_count;
            c.count        = new_count;
            c.stride_bytes = new_stride_bytes;
            c.alignment    = new_alignment;
            c.owns         = true;
            c.elem_size    = sizeof(T);
            c.type         = std::type_index(typeid(T));
        }
        [[nodiscard]] const ColumnDesc& desc(ColumnId id) const {
            return cols_.at(id);
        }
        void release_all() {
            for (auto& c : cols_) {
                if (c.owns && c.data) {
                    mr_->deallocate(c.data, c.bytes, c.alignment);
                }
                c.data  = nullptr;
                c.bytes = 0;
                c.count = 0;
            }
            cols_.clear();
        }

    private:
        std::pmr::memory_resource* mr_;
        std::vector<ColumnDesc> cols_;
        template <class T>
        static void check_params(size_t count, size_t stride_bytes, size_t alignment) {
            if (count == 0) {
                return;
            }
            assert(alignment % alignof(T) == 0);
            assert(std::has_single_bit(alignment));
            assert(stride_bytes % sizeof(T) == 0);
            constexpr size_t max = std::numeric_limits<size_t>::max();
            assert(stride_bytes <= max / count);
            (void) max;
            (void) stride_bytes;
            (void) alignment;
            (void) alignof(T);
        }
        template <class T>
        static void debug_check_type(const ColumnDesc& c) {
#ifndef NDEBUG
            if (c.elem_size != sizeof(T)) {
                assert(false && "bind<T> type mismatch");
            }
#else
            (void) c;
#endif
        }
        template <class T>
        static void debug_check_contiguous(const ColumnDesc& c) {
#ifndef NDEBUG
            if (c.stride_bytes != sizeof(T)) {
                assert(false && "bind<T> requires contiguous storage, use bind_strided<T>");
            }
#else
            (void) c;
#endif
        }
        template <class T>
        ColumnId emplace_desc(void* data, size_t bytes, size_t count, size_t stride_bytes, size_t alignment, bool owns) {
            ColumnDesc d;
            d.data         = data;
            d.bytes        = bytes;
            d.count        = count;
            d.stride_bytes = stride_bytes;
            d.alignment    = alignment;
            d.owns         = owns;
            d.elem_size    = sizeof(T);
            d.type         = std::type_index(typeid(T));
            cols_.push_back(d);
            return static_cast<ColumnId>(cols_.size() - 1);
        }
    };
} // namespace HinaPE

#endif // HINACLOTH_ALIGNED_ALLOCATOR_H

#ifndef HINACLOTH_ALIGNED_ALLOCATOR_H
#define HINACLOTH_ALIGNED_ALLOCATOR_H

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mdspan>
#include <memory_resource>
#include <new>
#include <span>
#include <type_traits>
#include <typeindex>
#include <vector>

namespace HinaPE {

    class AlignedResource final : public std::pmr::memory_resource {
    public:
        explicit AlignedResource(std::size_t alignment = 64, std::pmr::memory_resource* upstream = std::pmr::get_default_resource()) : align_(alignment), upstream_(upstream) {}

    private:
        std::size_t align_;
        std::pmr::memory_resource* upstream_;

        static std::size_t normalize_alignment(std::size_t requested) {
            constexpr std::size_t min_align = alignof(void*);
            requested                      = std::max(requested, min_align);
            if (!std::has_single_bit(requested)) {
                requested = std::bit_ceil(requested);
            }
            return requested;
        }

        void* do_allocate(std::size_t bytes, std::size_t alignment) override {
            std::size_t required = normalize_alignment(std::max(align_, alignment));
            if (required <= alignof(std::max_align_t)) {
                return upstream_->allocate(bytes == 0 ? sizeof(std::max_align_t) : bytes, required);
            }
#if defined(_MSC_VER)
            void* p = _aligned_malloc(bytes == 0 ? required : bytes, required);
            if (!p) {
                throw std::bad_alloc{};
            }
            return p;
#else
            void* p      = nullptr;
            const auto s = static_cast<std::size_t>(bytes == 0 ? required : bytes);
            if (posix_memalign(&p, required, s) != 0) {
                throw std::bad_alloc{};
            }
            return p;
#endif
        }

        void do_deallocate(void* p, std::size_t bytes, std::size_t alignment) override {
            if (!p) {
                return;
            }
            std::size_t required = normalize_alignment(std::max(align_, alignment));
            if (required <= alignof(std::max_align_t)) {
                upstream_->deallocate(p, bytes == 0 ? sizeof(std::max_align_t) : bytes, required);
                return;
            }
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
    inline constexpr ColumnId invalid_column_id = std::numeric_limits<ColumnId>::max();

    struct no_init_t {
    };
    inline constexpr no_init_t no_init{};

    template <class T>
    struct ColumnHandle {
        ColumnId id{invalid_column_id};
        size_t stride_bytes{sizeof(T)};
        size_t alignment{alignof(T)};
        [[nodiscard]] bool valid() const noexcept {
            return id != invalid_column_id;
        }
    };

    template <class T>
    struct ColumnView {
        using element_type = T;
        using mdspan_type  = std::mdspan<T, std::extents<size_t, std::dynamic_extent>, std::layout_stride>;

        T* data{nullptr};
        size_t count{0};
        size_t stride_bytes{sizeof(T)};

        [[nodiscard]] bool empty() const noexcept {
            return count == 0;
        }
        [[nodiscard]] bool contiguous() const noexcept {
            return stride_bytes == sizeof(T);
        }
        [[nodiscard]] mdspan_type mdspan() const {
            using ext = std::extents<size_t, std::dynamic_extent>;
            std::array<size_t, 1> strides{std::max<std::size_t>(1, stride_bytes / sizeof(T))};
            return mdspan_type(data, ext(count), strides);
        }
        [[nodiscard]] auto span() const {
            assert(contiguous() && "ColumnView::span requires contiguous storage");
            using Value = std::remove_cv_t<T>;
            if constexpr (std::is_const_v<T>) {
                return std::span<const Value>(data, count);
            } else {
                return std::span<Value>(data, count);
            }
        }
        [[nodiscard]] auto span() {
            static_assert(!std::is_const_v<T>, "span() mut overload requires non-const element type");
            assert(contiguous() && "ColumnView::span requires contiguous storage");
            return std::span<T>(data, count);
        }
    };

    class ColumnArena {
    public:
        explicit ColumnArena(std::pmr::memory_resource* mr, size_t expected_columns = 0) : mr_(mr) {
            if (expected_columns) {
                cols_.reserve(expected_columns);
            }
        }
        ~ColumnArena() {
            release_all();
        }
        ColumnArena(const ColumnArena&)            = delete;
        ColumnArena& operator=(const ColumnArena&) = delete;
        ColumnArena(ColumnArena&&)                 = delete;
        ColumnArena& operator=(ColumnArena&&)      = delete;

        void reserve(size_t column_capacity) {
            cols_.reserve(column_capacity);
        }

        template <class T>
        [[nodiscard]] ColumnHandle<T> make_column(size_t count, size_t stride_bytes = sizeof(T), size_t alignment = 64) {
            alignment          = check_params<T>(count, stride_bytes, alignment);
            const size_t bytes = stride_bytes * count;
            void* p            = count == 0 ? nullptr : mr_->allocate(bytes, alignment);
            if (count != 0) {
                std::memset(p, 0, bytes);
            }
            ColumnId id = emplace_desc<T>(p, bytes, count, stride_bytes, alignment, true);
            return ColumnHandle<T>{id, stride_bytes, alignment};
        }

        template <class T>
        [[nodiscard]] ColumnHandle<T> make_column(size_t count, no_init_t, size_t stride_bytes = sizeof(T), size_t alignment = 64) {
            alignment          = check_params<T>(count, stride_bytes, alignment);
            const size_t bytes = stride_bytes * count;
            void* p            = count == 0 ? nullptr : mr_->allocate(bytes, alignment);
            ColumnId id        = emplace_desc<T>(p, bytes, count, stride_bytes, alignment, true);
            return ColumnHandle<T>{id, stride_bytes, alignment};
        }

        template <class T>
        [[nodiscard]] ColumnHandle<T> map_external(T* ptr, size_t count, size_t stride_bytes = sizeof(T), size_t alignment = alignof(T)) {
            alignment = check_params<T>(count, stride_bytes, alignment);
            ColumnId id = emplace_desc<T>(ptr, stride_bytes * count, count, stride_bytes, alignment, false);
            return ColumnHandle<T>{id, stride_bytes, alignment};
        }

        template <class T>
        [[nodiscard]] ColumnView<T> view(ColumnHandle<T> handle) {
            auto& c = cols_.at(handle.id);
            debug_check_type<std::remove_cv_t<T>>(c);
            return ColumnView<T>{static_cast<T*>(c.data), c.count, c.stride_bytes};
        }

        template <class T>
        [[nodiscard]] ColumnView<const T> view_const(ColumnHandle<T> handle) const {
            const auto& c = cols_.at(handle.id);
            debug_check_type<std::remove_cv_t<T>>(c);
            return ColumnView<const T>{static_cast<const T*>(c.data), c.count, c.stride_bytes};
        }

        template <class T>
        void reallocate_preserve(ColumnHandle<T>& handle, size_t new_count, size_t new_stride_bytes, size_t new_alignment) {
            auto& c = cols_.at(handle.id);
            debug_check_type<std::remove_cv_t<T>>(c);
            new_alignment = check_params<T>(new_count, new_stride_bytes, new_alignment);

            if (new_count == 0) {
                if (c.owns && c.data) {
                    mr_->deallocate(c.data, c.bytes, c.alignment);
                }
                c.data             = nullptr;
                c.bytes            = 0;
                c.count            = 0;
                c.stride_bytes     = new_stride_bytes;
                c.alignment        = new_alignment;
                handle.stride_bytes = new_stride_bytes;
                handle.alignment    = new_alignment;
                return;
            }

            void* p = mr_->allocate(new_stride_bytes * new_count, new_alignment);
            const size_t copy_elems = std::min(c.count, new_count);
            if (copy_elems > 0 && c.data) {
                if (const size_t elem = sizeof(T); c.stride_bytes == elem && new_stride_bytes == elem) {
                    std::memcpy(p, c.data, copy_elems * elem);
                } else {
                    auto* src = static_cast<const std::byte*>(c.data);
                    auto* dst = static_cast<std::byte*>(p);
                    for (size_t i = 0; i < copy_elems; ++i) {
                        std::memcpy(dst + i * new_stride_bytes, src + i * c.stride_bytes, elem);
                    }
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
            handle.stride_bytes = new_stride_bytes;
            handle.alignment    = new_alignment;
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
        static size_t check_params(size_t count, size_t stride_bytes, size_t alignment) {
            alignment = std::max(alignment, alignof(T));
            if (!std::has_single_bit(alignment)) {
                alignment = std::bit_ceil(alignment);
            }
            if (count == 0) {
                return alignment;
            }
            assert(stride_bytes % sizeof(T) == 0);
            constexpr size_t max = std::numeric_limits<size_t>::max();
            assert(stride_bytes <= max / count);
            (void) max;
            return alignment;
        }

        template <class T>
        static void debug_check_type(const ColumnDesc& c) {
#ifndef NDEBUG
            if (c.elem_size != sizeof(T)) {
                assert(false && "Column type mismatch");
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
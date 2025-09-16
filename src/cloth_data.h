#ifndef HINACLOTH_CLOTH_DATA_H
#define HINACLOTH_CLOTH_DATA_H

#include "aligned_allocator.h"
#include <cstdint>
#include <limits>
#include <mdspan>
#include <memory_resource>

namespace HinaPE {

    using u8  = std::uint8_t;
    using u16 = std::uint16_t;
    using u32 = std::uint32_t;

    struct ParticleView {
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> px, py, pz;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> vx, vy, vz;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> inv_mass;
        std::mdspan<u8, std::extents<size_t, std::dynamic_extent>> pinned;
        size_t n{};
    };

    struct DistanceView {
        std::mdspan<u32, std::extents<size_t, std::dynamic_extent>> i, j;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> rest, compliance, lambda, alpha;
        std::mdspan<u8, std::extents<size_t, std::dynamic_extent>> color;
        size_t m{};
    };

    struct TrianglesView {
        std::mdspan<u32, std::extents<size_t, std::dynamic_extent>> f0, f1, f2;
        size_t n{};
    };

    struct BendingView {
        std::mdspan<u32, std::extents<size_t, std::dynamic_extent>> e0, e1, e2, e3;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> rest_angle, stiffness, lambda, alpha;
        std::mdspan<u8, std::extents<size_t, std::dynamic_extent>> color;
        size_t m{};
    };

    struct TriElasticView {
        std::mdspan<u32, std::extents<size_t, std::dynamic_extent>> f0, f1, f2;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> area;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> DmInv00, DmInv01, DmInv10, DmInv11;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> youngs, poisson;
        std::mdspan<float, std::extents<size_t, std::dynamic_extent>> R00, R01, R10, R11;
        size_t m{};
    };

    class ClothData {
    public:
        explicit ClothData(std::size_t alignment = 64) : mem_(alignment), arena_(&mem_) {}

        ClothData(const ClothData&)            = delete;
        ClothData& operator=(const ClothData&) = delete;

        void allocate_particles(size_t n) {
            n_      = n;
            id_px_  = arena_.make_column<float>(n);
            id_py_  = arena_.make_column<float>(n);
            id_pz_  = arena_.make_column<float>(n);
            id_vx_  = arena_.make_column<float>(n);
            id_vy_  = arena_.make_column<float>(n);
            id_vz_  = arena_.make_column<float>(n);
            id_w_   = arena_.make_column<float>(n);
            id_pin_ = arena_.make_column<u8>(n);
        }

        void allocate_distance(size_t m) {
            m_edge_   = m;
            id_I_     = arena_.make_column<u32>(m);
            id_J_     = arena_.make_column<u32>(m);
            id_rest_  = arena_.make_column<float>(m);
            id_comp_  = arena_.make_column<float>(m);
            id_lam_   = arena_.make_column<float>(m);
            id_alpha_ = arena_.make_column<float>(m);
            id_color_ = arena_.make_column<u8>(m);
        }

        void allocate_triangles(size_t nfaces) {
            n_face_ = nfaces;
            id_f0_  = arena_.make_column<u32>(nfaces);
            id_f1_  = arena_.make_column<u32>(nfaces);
            id_f2_  = arena_.make_column<u32>(nfaces);
        }

        void allocate_bending(size_t m) {
            m_bend_    = m;
            id_be0_    = arena_.make_column<u32>(m);
            id_be1_    = arena_.make_column<u32>(m);
            id_be2_    = arena_.make_column<u32>(m);
            id_be3_    = arena_.make_column<u32>(m);
            id_brest_  = arena_.make_column<float>(m);
            id_bstiff_ = arena_.make_column<float>(m);
            id_blam_   = arena_.make_column<float>(m);
            id_balpha_ = arena_.make_column<float>(m);
            id_bcolor_ = arena_.make_column<u8>(m);
        }

        void allocate_tri_elastic(size_t m) {
            m_tre_   = m;
            id_tef0_ = arena_.make_column<u32>(m);
            id_tef1_ = arena_.make_column<u32>(m);
            id_tef2_ = arena_.make_column<u32>(m);
            id_area_ = arena_.make_column<float>(m);
            id_dm00_ = arena_.make_column<float>(m);
            id_dm01_ = arena_.make_column<float>(m);
            id_dm10_ = arena_.make_column<float>(m);
            id_dm11_ = arena_.make_column<float>(m);
            id_E_    = arena_.make_column<float>(m);
            id_nu_   = arena_.make_column<float>(m);
            id_R00_  = arena_.make_column<float>(m);
            id_R01_  = arena_.make_column<float>(m);
            id_R10_  = arena_.make_column<float>(m);
            id_R11_  = arena_.make_column<float>(m);
        }

        ParticleView particles() {
            ParticleView V{};
            V.px       = arena_.bind<float>(id_px_);
            V.py       = arena_.bind<float>(id_py_);
            V.pz       = arena_.bind<float>(id_pz_);
            V.vx       = arena_.bind<float>(id_vx_);
            V.vy       = arena_.bind<float>(id_vy_);
            V.vz       = arena_.bind<float>(id_vz_);
            V.inv_mass = arena_.bind<float>(id_w_);
            V.pinned   = arena_.bind<u8>(id_pin_);
            V.n        = n_;
            return V;
        }

        DistanceView distance() {
            DistanceView E{};
            E.i          = arena_.bind<u32>(id_I_);
            E.j          = arena_.bind<u32>(id_J_);
            E.rest       = arena_.bind<float>(id_rest_);
            E.compliance = arena_.bind<float>(id_comp_);
            E.lambda     = arena_.bind<float>(id_lam_);
            E.alpha      = arena_.bind<float>(id_alpha_);
            E.color      = arena_.bind<u8>(id_color_);
            E.m          = m_edge_;
            return E;
        }

        TrianglesView triangles() {
            TrianglesView T{};
            T.f0 = arena_.bind<u32>(id_f0_);
            T.f1 = arena_.bind<u32>(id_f1_);
            T.f2 = arena_.bind<u32>(id_f2_);
            T.n  = n_face_;
            return T;
        }

        BendingView bending() {
            BendingView B{};
            B.e0         = arena_.bind<u32>(id_be0_);
            B.e1         = arena_.bind<u32>(id_be1_);
            B.e2         = arena_.bind<u32>(id_be2_);
            B.e3         = arena_.bind<u32>(id_be3_);
            B.rest_angle = arena_.bind<float>(id_brest_);
            B.stiffness  = arena_.bind<float>(id_bstiff_);
            B.lambda     = arena_.bind<float>(id_blam_);
            B.alpha      = arena_.bind<float>(id_balpha_);
            B.color      = arena_.bind<u8>(id_bcolor_);
            B.m          = m_bend_;
            return B;
        }

        TriElasticView tri_elastic() {
            TriElasticView R{};
            R.f0      = arena_.bind<u32>(id_tef0_);
            R.f1      = arena_.bind<u32>(id_tef1_);
            R.f2      = arena_.bind<u32>(id_tef2_);
            R.area    = arena_.bind<float>(id_area_);
            R.DmInv00 = arena_.bind<float>(id_dm00_);
            R.DmInv01 = arena_.bind<float>(id_dm01_);
            R.DmInv10 = arena_.bind<float>(id_dm10_);
            R.DmInv11 = arena_.bind<float>(id_dm11_);
            R.youngs  = arena_.bind<float>(id_E_);
            R.poisson = arena_.bind<float>(id_nu_);
            R.R00     = arena_.bind<float>(id_R00_);
            R.R01     = arena_.bind<float>(id_R01_);
            R.R10     = arena_.bind<float>(id_R10_);
            R.R11     = arena_.bind<float>(id_R11_);
            R.m       = m_tre_;
            return R;
        }

        [[nodiscard]] const ColumnArena& arena() const {
            return arena_;
        }
        ColumnArena& arena() {
            return arena_;
        }

        [[nodiscard]] size_t num_particles() const {
            return n_;
        }
        [[nodiscard]] size_t num_edges() const {
            return m_edge_;
        }
        [[nodiscard]] size_t num_faces() const {
            return n_face_;
        }
        [[nodiscard]] size_t num_bending() const {
            return m_bend_;
        }
        [[nodiscard]] size_t num_tri_elastic() const {
            return m_tre_;
        }

    private:
        static constexpr ColumnId invalid_ = std::numeric_limits<ColumnId>::max();

        AlignedResource mem_;
        ColumnArena arena_;

        size_t n_{0}, m_edge_{0}, n_face_{0}, m_bend_{0}, m_tre_{0};

        ColumnId id_px_{invalid_}, id_py_{invalid_}, id_pz_{invalid_};
        ColumnId id_vx_{invalid_}, id_vy_{invalid_}, id_vz_{invalid_};
        ColumnId id_w_{invalid_}, id_pin_{invalid_};

        ColumnId id_I_{invalid_}, id_J_{invalid_};
        ColumnId id_rest_{invalid_}, id_comp_{invalid_}, id_lam_{invalid_}, id_alpha_{invalid_};
        ColumnId id_color_{invalid_};

        ColumnId id_f0_{invalid_}, id_f1_{invalid_}, id_f2_{invalid_};

        ColumnId id_be0_{invalid_}, id_be1_{invalid_}, id_be2_{invalid_}, id_be3_{invalid_};
        ColumnId id_brest_{invalid_}, id_bstiff_{invalid_}, id_blam_{invalid_}, id_balpha_{invalid_};
        ColumnId id_bcolor_{invalid_};

        ColumnId id_tef0_{invalid_}, id_tef1_{invalid_}, id_tef2_{invalid_};
        ColumnId id_area_{invalid_};
        ColumnId id_dm00_{invalid_}, id_dm01_{invalid_}, id_dm10_{invalid_}, id_dm11_{invalid_};
        ColumnId id_E_{invalid_}, id_nu_{invalid_};
        ColumnId id_R00_{invalid_}, id_R01_{invalid_}, id_R10_{invalid_}, id_R11_{invalid_};
    };

    inline ParticleView make_particle_view(ClothData& D) {
        return D.particles();
    }
    inline DistanceView make_distance_view(ClothData& D) {
        return D.distance();
    }
    inline TrianglesView make_triangles_view(ClothData& D) {
        return D.triangles();
    }
    inline BendingView make_bending_view(ClothData& D) {
        return D.bending();
    }
    inline TriElasticView make_trielastic_view(ClothData& D) {
        return D.tri_elastic();
    }

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_H

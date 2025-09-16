#ifndef HINACLOTH_CLOTH_DATA_H
#define HINACLOTH_CLOTH_DATA_H

#include "aligned_allocator.h"
#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <type_traits>

namespace HinaPE {

    using u8  = std::uint8_t;
    using u16 = std::uint16_t;
    using u32 = std::uint32_t;

    template <class FloatT, class FlagT>
    struct ParticleViewT {
        ColumnView<FloatT> px, py, pz;
        ColumnView<FloatT> vx, vy, vz;
        ColumnView<FloatT> inv_mass;
        ColumnView<FlagT> pinned;
        size_t n{};
    };
    using ParticleView      = ParticleViewT<float, u8>;
    using ParticleConstView = ParticleViewT<const float, const u8>;

    template <class IndexT, class ScalarT, class FlagT>
    struct DistanceViewT {
        ColumnView<IndexT> i, j;
        ColumnView<ScalarT> rest, compliance, lambda, alpha;
        ColumnView<FlagT> color;
        size_t m{};
    };
    using DistanceView      = DistanceViewT<u32, float, u8>;
    using DistanceConstView = DistanceViewT<const u32, const float, const u8>;

    template <class IndexT>
    struct TrianglesViewT {
        ColumnView<IndexT> f0, f1, f2;
        size_t n{};
    };
    using TrianglesView      = TrianglesViewT<u32>;
    using TrianglesConstView = TrianglesViewT<const u32>;

    template <class IndexT, class ScalarT, class FlagT>
    struct BendingViewT {
        ColumnView<IndexT> e0, e1, e2, e3;
        ColumnView<ScalarT> rest_angle, stiffness, lambda, alpha;
        ColumnView<FlagT> color;
        size_t m{};
    };
    using BendingView      = BendingViewT<u32, float, u8>;
    using BendingConstView = BendingViewT<const u32, const float, const u8>;

    template <class IndexT, class ScalarT>
    struct TriElasticViewT {
        ColumnView<IndexT> f0, f1, f2;
        ColumnView<ScalarT> area;
        ColumnView<ScalarT> DmInv00, DmInv01, DmInv10, DmInv11;
        ColumnView<ScalarT> youngs, poisson;
        ColumnView<ScalarT> R00, R01, R10, R11;
        size_t m{};
    };
    using TriElasticView      = TriElasticViewT<u32, float>;
    using TriElasticConstView = TriElasticViewT<const u32, const float>;

    class ClothData {
    public:
        explicit ClothData(std::size_t alignment = 64, size_t column_hint = 48) : mem_(alignment), arena_(&mem_, column_hint), alignment_(std::bit_ceil(std::max(alignment, std::size_t{alignof(float)}))) {}

        ClothData(const ClothData&)            = delete;
        ClothData& operator=(const ClothData&) = delete;

        void allocate_particles(size_t n) {
            n_ = n;
            ensure_column(px_, n);
            ensure_column(py_, n);
            ensure_column(pz_, n);
            ensure_column(vx_, n);
            ensure_column(vy_, n);
            ensure_column(vz_, n);
            ensure_column(inv_mass_, n);
            ensure_column(pinned_, n, sizeof(u8), alignment_);
        }

        void allocate_distance(size_t m) {
            m_edge_ = m;
            ensure_column(edge_i_, m, sizeof(u32), alignment_);
            ensure_column(edge_j_, m, sizeof(u32), alignment_);
            ensure_column(rest_, m);
            ensure_column(compliance_, m);
            ensure_column(lambda_, m);
            ensure_column(alpha_, m);
            ensure_column(edge_color_, m, sizeof(u8), alignment_);
        }

        void allocate_triangles(size_t nfaces) {
            n_face_ = nfaces;
            ensure_column(face_f0_, nfaces, sizeof(u32), alignment_);
            ensure_column(face_f1_, nfaces, sizeof(u32), alignment_);
            ensure_column(face_f2_, nfaces, sizeof(u32), alignment_);
        }

        void allocate_bending(size_t m) {
            m_bend_ = m;
            ensure_column(bend_e0_, m, sizeof(u32), alignment_);
            ensure_column(bend_e1_, m, sizeof(u32), alignment_);
            ensure_column(bend_e2_, m, sizeof(u32), alignment_);
            ensure_column(bend_e3_, m, sizeof(u32), alignment_);
            ensure_column(bend_rest_angle_, m);
            ensure_column(bend_stiffness_, m);
            ensure_column(bend_lambda_, m);
            ensure_column(bend_alpha_, m);
            ensure_column(bend_color_, m, sizeof(u8), alignment_);
        }

        void allocate_tri_elastic(size_t m) {
            m_tre_ = m;
            ensure_column(tri_f0_, m, sizeof(u32), alignment_);
            ensure_column(tri_f1_, m, sizeof(u32), alignment_);
            ensure_column(tri_f2_, m, sizeof(u32), alignment_);
            ensure_column(tri_area_, m);
            ensure_column(tri_dm00_, m);
            ensure_column(tri_dm01_, m);
            ensure_column(tri_dm10_, m);
            ensure_column(tri_dm11_, m);
            ensure_column(tri_E_, m);
            ensure_column(tri_nu_, m);
            ensure_column(tri_R00_, m);
            ensure_column(tri_R01_, m);
            ensure_column(tri_R10_, m);
            ensure_column(tri_R11_, m);
        }

        ParticleView particles() {
            ParticleView V{};
            V.px       = arena_.view(px_);
            V.py       = arena_.view(py_);
            V.pz       = arena_.view(pz_);
            V.vx       = arena_.view(vx_);
            V.vy       = arena_.view(vy_);
            V.vz       = arena_.view(vz_);
            V.inv_mass = arena_.view(inv_mass_);
            V.pinned   = arena_.view(pinned_);
            V.n        = n_;
            return V;
        }
        ParticleConstView particles() const {
            ParticleConstView V{};
            V.px       = arena_.view_const(px_);
            V.py       = arena_.view_const(py_);
            V.pz       = arena_.view_const(pz_);
            V.vx       = arena_.view_const(vx_);
            V.vy       = arena_.view_const(vy_);
            V.vz       = arena_.view_const(vz_);
            V.inv_mass = arena_.view_const(inv_mass_);
            V.pinned   = arena_.view_const(pinned_);
            V.n        = n_;
            return V;
        }

        DistanceView distance() {
            DistanceView E{};
            E.i          = arena_.view(edge_i_);
            E.j          = arena_.view(edge_j_);
            E.rest       = arena_.view(rest_);
            E.compliance = arena_.view(compliance_);
            E.lambda     = arena_.view(lambda_);
            E.alpha      = arena_.view(alpha_);
            E.color      = arena_.view(edge_color_);
            E.m          = m_edge_;
            return E;
        }
        DistanceConstView distance() const {
            DistanceConstView E{};
            E.i          = arena_.view_const(edge_i_);
            E.j          = arena_.view_const(edge_j_);
            E.rest       = arena_.view_const(rest_);
            E.compliance = arena_.view_const(compliance_);
            E.lambda     = arena_.view_const(lambda_);
            E.alpha      = arena_.view_const(alpha_);
            E.color      = arena_.view_const(edge_color_);
            E.m          = m_edge_;
            return E;
        }

        TrianglesView triangles() {
            TrianglesView T{};
            T.f0 = arena_.view(face_f0_);
            T.f1 = arena_.view(face_f1_);
            T.f2 = arena_.view(face_f2_);
            T.n  = n_face_;
            return T;
        }
        TrianglesConstView triangles() const {
            TrianglesConstView T{};
            T.f0 = arena_.view_const(face_f0_);
            T.f1 = arena_.view_const(face_f1_);
            T.f2 = arena_.view_const(face_f2_);
            T.n  = n_face_;
            return T;
        }

        BendingView bending() {
            BendingView B{};
            B.e0         = arena_.view(bend_e0_);
            B.e1         = arena_.view(bend_e1_);
            B.e2         = arena_.view(bend_e2_);
            B.e3         = arena_.view(bend_e3_);
            B.rest_angle = arena_.view(bend_rest_angle_);
            B.stiffness  = arena_.view(bend_stiffness_);
            B.lambda     = arena_.view(bend_lambda_);
            B.alpha      = arena_.view(bend_alpha_);
            B.color      = arena_.view(bend_color_);
            B.m          = m_bend_;
            return B;
        }
        BendingConstView bending() const {
            BendingConstView B{};
            B.e0         = arena_.view_const(bend_e0_);
            B.e1         = arena_.view_const(bend_e1_);
            B.e2         = arena_.view_const(bend_e2_);
            B.e3         = arena_.view_const(bend_e3_);
            B.rest_angle = arena_.view_const(bend_rest_angle_);
            B.stiffness  = arena_.view_const(bend_stiffness_);
            B.lambda     = arena_.view_const(bend_lambda_);
            B.alpha      = arena_.view_const(bend_alpha_);
            B.color      = arena_.view_const(bend_color_);
            B.m          = m_bend_;
            return B;
        }

        TriElasticView tri_elastic() {
            TriElasticView R{};
            R.f0      = arena_.view(tri_f0_);
            R.f1      = arena_.view(tri_f1_);
            R.f2      = arena_.view(tri_f2_);
            R.area    = arena_.view(tri_area_);
            R.DmInv00 = arena_.view(tri_dm00_);
            R.DmInv01 = arena_.view(tri_dm01_);
            R.DmInv10 = arena_.view(tri_dm10_);
            R.DmInv11 = arena_.view(tri_dm11_);
            R.youngs  = arena_.view(tri_E_);
            R.poisson = arena_.view(tri_nu_);
            R.R00     = arena_.view(tri_R00_);
            R.R01     = arena_.view(tri_R01_);
            R.R10     = arena_.view(tri_R10_);
            R.R11     = arena_.view(tri_R11_);
            R.m       = m_tre_;
            return R;
        }
        TriElasticConstView tri_elastic() const {
            TriElasticConstView R{};
            R.f0      = arena_.view_const(tri_f0_);
            R.f1      = arena_.view_const(tri_f1_);
            R.f2      = arena_.view_const(tri_f2_);
            R.area    = arena_.view_const(tri_area_);
            R.DmInv00 = arena_.view_const(tri_dm00_);
            R.DmInv01 = arena_.view_const(tri_dm01_);
            R.DmInv10 = arena_.view_const(tri_dm10_);
            R.DmInv11 = arena_.view_const(tri_dm11_);
            R.youngs  = arena_.view_const(tri_E_);
            R.poisson = arena_.view_const(tri_nu_);
            R.R00     = arena_.view_const(tri_R00_);
            R.R01     = arena_.view_const(tri_R01_);
            R.R10     = arena_.view_const(tri_R10_);
            R.R11     = arena_.view_const(tri_R11_);
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
        template <class T>
        void ensure_column(ColumnHandle<T>& handle, size_t count, size_t stride_bytes = sizeof(T), size_t alignment = alignof(T)) {
            alignment = std::bit_ceil(std::max(alignment_, std::max(alignment, std::size_t{alignof(T)})));
            if (!handle.valid()) {
                handle = arena_.make_column<T>(count, stride_bytes, alignment);
            } else {
                arena_.reallocate_preserve(handle, count, stride_bytes, alignment);
            }
        }

        AlignedResource mem_;
        ColumnArena arena_;
        std::size_t alignment_;

        size_t n_{0}, m_edge_{0}, n_face_{0}, m_bend_{0}, m_tre_{0};

        ColumnHandle<float> px_{}, py_{}, pz_{};
        ColumnHandle<float> vx_{}, vy_{}, vz_{};
        ColumnHandle<float> inv_mass_{};
        ColumnHandle<u8> pinned_{};

        ColumnHandle<u32> edge_i_{}, edge_j_{};
        ColumnHandle<float> rest_{}, compliance_{}, lambda_{}, alpha_{};
        ColumnHandle<u8> edge_color_{};

        ColumnHandle<u32> face_f0_{}, face_f1_{}, face_f2_{};

        ColumnHandle<u32> bend_e0_{}, bend_e1_{}, bend_e2_{}, bend_e3_{};
        ColumnHandle<float> bend_rest_angle_{}, bend_stiffness_{}, bend_lambda_{}, bend_alpha_{};
        ColumnHandle<u8> bend_color_{};

        ColumnHandle<u32> tri_f0_{}, tri_f1_{}, tri_f2_{};
        ColumnHandle<float> tri_area_{};
        ColumnHandle<float> tri_dm00_{}, tri_dm01_{}, tri_dm10_{}, tri_dm11_{};
        ColumnHandle<float> tri_E_{}, tri_nu_{};
        ColumnHandle<float> tri_R00_{}, tri_R01_{}, tri_R10_{}, tri_R11_{};
    };

    inline ParticleView make_particle_view(ClothData& D) {
        return D.particles();
    }
    inline ParticleConstView make_particle_view(const ClothData& D) {
        return D.particles();
    }
    inline DistanceView make_distance_view(ClothData& D) {
        return D.distance();
    }
    inline DistanceConstView make_distance_view(const ClothData& D) {
        return D.distance();
    }
    inline TrianglesView make_triangles_view(ClothData& D) {
        return D.triangles();
    }
    inline TrianglesConstView make_triangles_view(const ClothData& D) {
        return D.triangles();
    }
    inline BendingView make_bending_view(ClothData& D) {
        return D.bending();
    }
    inline BendingConstView make_bending_view(const ClothData& D) {
        return D.bending();
    }
    inline TriElasticView make_trielastic_view(ClothData& D) {
        return D.tri_elastic();
    }
    inline TriElasticConstView make_trielastic_view(const ClothData& D) {
        return D.tri_elastic();
    }

} // namespace HinaPE

#endif // HINACLOTH_CLOTH_DATA_H

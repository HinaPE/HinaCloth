#include "cloth_data.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace HinaPE;

namespace {

struct Entry {
    bool ok{false};
    std::string message;
};

struct Section {
    std::string name;
    std::vector<Entry> entries;
};

void record(Section& section, bool ok, std::string message) {
    section.entries.push_back(Entry{ok, std::move(message)});
}

bool isAligned(const void* ptr, std::size_t alignment) {
    if (alignment == 0) {
        return true;
    }
    if (ptr == nullptr) {
        return false;
    }
    const auto addr = reinterpret_cast<std::uintptr_t>(ptr);
    return (addr % alignment) == 0;
}

std::string mark(bool ok) {
    return ok ? "OK" : "ERR";
}

} // namespace

int main() {
    std::vector<Section> sections;
    sections.reserve(8);

    // Particle storage layout and access
    {
        Section sec{"Particle Columns"};
        ClothData cloth{};
        cloth.allocate_particles(8);
        auto particles = cloth.particles();

        record(sec, cloth.num_particles() == 8, "num_particles reflects allocated count (expect 8)");
        record(sec, particles.n == 8, "particles view exposes 8 entries");

        record(sec, particles.px.contiguous(), "px column uses contiguous float storage");
        record(sec, particles.py.contiguous(), "py column uses contiguous float storage");
        record(sec, particles.pz.contiguous(), "pz column uses contiguous float storage");
        record(sec, particles.vx.contiguous(), "vx column uses contiguous float storage");
        record(sec, particles.vy.contiguous(), "vy column uses contiguous float storage");
        record(sec, particles.vz.contiguous(), "vz column uses contiguous float storage");
        record(sec, particles.inv_mass.contiguous(), "inv_mass column uses contiguous float storage");
        record(sec, particles.pinned.contiguous(), "pinned column uses contiguous byte storage");

        record(sec, isAligned(particles.px.data, 64), "px buffer is 64-byte aligned");
        record(sec, isAligned(particles.py.data, 64), "py buffer is 64-byte aligned");
        record(sec, isAligned(particles.pz.data, 64), "pz buffer is 64-byte aligned");
        record(sec, isAligned(particles.vx.data, 64), "vx buffer is 64-byte aligned");
        record(sec, isAligned(particles.vy.data, 64), "vy buffer is 64-byte aligned");
        record(sec, isAligned(particles.vz.data, 64), "vz buffer is 64-byte aligned");
        record(sec, isAligned(particles.inv_mass.data, 64), "inv_mass buffer is 64-byte aligned");
        record(sec, isAligned(particles.pinned.data, 64), "pinned buffer is 64-byte aligned");

        record(sec, particles.px.data != particles.py.data, "px and py columns do not alias");
        record(sec, particles.py.data != particles.pz.data, "py and pz columns do not alias");
        record(sec, particles.vx.data != particles.vy.data, "vx and vy columns do not alias");

        auto px_span       = particles.px.span();
        auto py_span       = particles.py.span();
        auto pz_span       = particles.pz.span();
        auto vx_span       = particles.vx.span();
        auto vy_span       = particles.vy.span();
        auto vz_span       = particles.vz.span();
        auto inv_mass_span = particles.inv_mass.span();
        auto pinned_span   = particles.pinned.span();

        for (std::size_t i = 0; i < particles.n; ++i) {
            px_span[i]       = static_cast<float>(i * 1.0f);
            py_span[i]       = static_cast<float>(i * 2.0f);
            pz_span[i]       = static_cast<float>(i * 3.0f);
            vx_span[i]       = static_cast<float>(10.0f + i);
            vy_span[i]       = static_cast<float>(20.0f + i);
            vz_span[i]       = static_cast<float>(30.0f + i);
            inv_mass_span[i] = 1.0f / static_cast<float>(i + 1);
            pinned_span[i]   = static_cast<u8>(i % 2);
        }

        const auto particles_const = std::as_const(cloth).particles();
        auto px_const              = particles_const.px.span();
        auto py_const              = particles_const.py.span();
        auto pz_const              = particles_const.pz.span();
        auto vx_const              = particles_const.vx.span();
        auto vy_const              = particles_const.vy.span();
        auto vz_const              = particles_const.vz.span();
        auto inv_mass_const        = particles_const.inv_mass.span();
        auto pinned_const          = particles_const.pinned.span();

        bool data_ok = true;
        for (std::size_t i = 0; i < particles.n; ++i) {
            data_ok = data_ok && px_const[i] == static_cast<float>(i * 1.0f) && py_const[i] == static_cast<float>(i * 2.0f)
                && pz_const[i] == static_cast<float>(i * 3.0f) && vx_const[i] == static_cast<float>(10.0f + i)
                && vy_const[i] == static_cast<float>(20.0f + i) && vz_const[i] == static_cast<float>(30.0f + i)
                && inv_mass_const[i] == 1.0f / static_cast<float>(i + 1) && pinned_const[i] == static_cast<u8>(i % 2);
        }
        record(sec, data_ok, "particle columns retain written values across const views");

        sections.push_back(std::move(sec));
    }

    // Reallocation should preserve existing particle data and extend capacity.
    {
        Section sec{"Particle Reallocation"};
        ClothData cloth{};
        cloth.allocate_particles(5);
        {
            auto particles = cloth.particles();
            auto px        = particles.px.span();
            auto py        = particles.py.span();
            auto pinned    = particles.pinned.span();
            for (std::size_t i = 0; i < particles.n; ++i) {
                px[i]     = 100.0f + static_cast<float>(i);
                py[i]     = 200.0f + static_cast<float>(i);
                pinned[i] = static_cast<u8>(1);
            }
        }

        cloth.allocate_particles(9);
        auto particles = cloth.particles();
        record(sec, cloth.num_particles() == 9, "num_particles updates after growth reallocation");
        record(sec, particles.n == 9, "view reflects new particle count");
        record(sec, isAligned(particles.px.data, 64), "px buffer remains 64-byte aligned after reallocation");

        auto px = particles.px.span();
        auto py = particles.py.span();
        auto pinned = particles.pinned.span();
        bool preserved = true;
        for (std::size_t i = 0; i < 5; ++i) {
            preserved = preserved && px[i] == 100.0f + static_cast<float>(i) && py[i] == 200.0f + static_cast<float>(i)
                && pinned[i] == static_cast<u8>(1);
        }
        record(sec, preserved, "existing particle data survives growth reallocation");

        for (std::size_t i = 5; i < particles.n; ++i) {
            px[i]     = 300.0f + static_cast<float>(i);
            py[i]     = 400.0f + static_cast<float>(i);
            pinned[i] = static_cast<u8>(0);
        }

        bool write_all = true;
        for (std::size_t i = 0; i < particles.n; ++i) {
            write_all = write_all && px[i] == ((i < 5) ? (100.0f + static_cast<float>(i)) : (300.0f + static_cast<float>(i)));
        }
        record(sec, write_all, "new particle slots are writable after growth");

        cloth.allocate_particles(0);
        record(sec, cloth.num_particles() == 0, "num_particles returns zero after releasing storage");
        record(sec, cloth.particles().n == 0, "particle view shows zero entries after release");

        sections.push_back(std::move(sec));
    }

    // Distance constraint storage
    {
        Section sec{"Distance Columns"};
        ClothData cloth{};
        cloth.allocate_distance(6);
        auto dist = cloth.distance();
        record(sec, cloth.num_edges() == 6, "num_edges matches allocated distance constraints");
        record(sec, dist.m == 6, "distance view exposes 6 constraints");
        record(sec, dist.i.contiguous() && dist.j.contiguous(), "distance index buffers are contiguous");
        record(sec, dist.rest.contiguous(), "rest lengths column uses contiguous storage");
        record(sec, dist.compliance.contiguous(), "compliance column uses contiguous storage");
        record(sec, dist.lambda.contiguous(), "lambda column uses contiguous storage");
        record(sec, dist.alpha.contiguous(), "alpha column uses contiguous storage");
        record(sec, dist.color.contiguous(), "color column uses contiguous storage");

        record(sec, isAligned(dist.i.data, 64), "distance i buffer is 64-byte aligned");
        record(sec, isAligned(dist.j.data, 64), "distance j buffer is 64-byte aligned");
        record(sec, isAligned(dist.rest.data, 64), "distance rest buffer is 64-byte aligned");
        record(sec, isAligned(dist.color.data, 64), "distance color buffer is 64-byte aligned");

        auto i_span      = dist.i.span();
        auto j_span      = dist.j.span();
        auto rest_span   = dist.rest.span();
        auto comp_span   = dist.compliance.span();
        auto lambda_span = dist.lambda.span();
        auto alpha_span  = dist.alpha.span();
        auto color_span  = dist.color.span();

        for (std::size_t idx = 0; idx < dist.m; ++idx) {
            i_span[idx]      = static_cast<u32>(idx);
            j_span[idx]      = static_cast<u32>(idx + 1u);
            rest_span[idx]   = 0.5f + static_cast<float>(idx) * 0.1f;
            comp_span[idx]   = static_cast<float>(idx) * 0.01f;
            lambda_span[idx] = static_cast<float>(idx) * 0.02f;
            alpha_span[idx]  = static_cast<float>(idx) * 0.03f;
            color_span[idx]  = static_cast<u8>(idx % 4u);
        }

        const auto dist_const = std::as_const(cloth).distance();
        auto i_c              = dist_const.i.span();
        auto j_c              = dist_const.j.span();
        auto rest_c           = dist_const.rest.span();
        auto comp_c           = dist_const.compliance.span();
        auto lambda_c         = dist_const.lambda.span();
        auto alpha_c          = dist_const.alpha.span();
        auto color_c          = dist_const.color.span();

        bool dist_ok = true;
        for (std::size_t idx = 0; idx < dist.m; ++idx) {
            dist_ok = dist_ok && i_c[idx] == static_cast<u32>(idx) && j_c[idx] == static_cast<u32>(idx + 1u)
                && rest_c[idx] == (0.5f + static_cast<float>(idx) * 0.1f)
                && comp_c[idx] == static_cast<float>(idx) * 0.01f
                && lambda_c[idx] == static_cast<float>(idx) * 0.02f
                && alpha_c[idx] == static_cast<float>(idx) * 0.03f
                && color_c[idx] == static_cast<u8>(idx % 4u);
        }
        record(sec, dist_ok, "distance constraint data is preserved and accessible via const view");

        sections.push_back(std::move(sec));
    }

    // Triangle index buffers
    {
        Section sec{"Triangle Columns"};
        ClothData cloth{};
        cloth.allocate_triangles(4);
        auto tri = cloth.triangles();
        record(sec, cloth.num_faces() == 4, "num_faces matches allocation");
        record(sec, tri.n == 4, "triangle view exposes 4 faces");
        record(sec, tri.f0.contiguous() && tri.f1.contiguous() && tri.f2.contiguous(), "triangle index buffers are contiguous");
        record(sec, isAligned(tri.f0.data, 64), "triangle f0 buffer is 64-byte aligned");

        auto f0 = tri.f0.span();
        auto f1 = tri.f1.span();
        auto f2 = tri.f2.span();
        for (std::size_t idx = 0; idx < tri.n; ++idx) {
            f0[idx] = static_cast<u32>(idx * 3);
            f1[idx] = static_cast<u32>(idx * 3 + 1);
            f2[idx] = static_cast<u32>(idx * 3 + 2);
        }

        const auto tri_const = std::as_const(cloth).triangles();
        auto f0_c            = tri_const.f0.span();
        bool tri_ok          = true;
        for (std::size_t idx = 0; idx < tri.n; ++idx) {
            tri_ok = tri_ok && f0_c[idx] == static_cast<u32>(idx * 3);
        }
        record(sec, tri_ok, "triangle indices persist across const view");

        sections.push_back(std::move(sec));
    }

    // Bending constraint buffers
    {
        Section sec{"Bending Columns"};
        ClothData cloth{};
        cloth.allocate_bending(3);
        auto bend = cloth.bending();
        record(sec, cloth.num_bending() == 3, "num_bending matches allocation");
        record(sec, bend.m == 3, "bending view exposes 3 constraints");
        record(sec, bend.e0.contiguous() && bend.e3.contiguous(), "bending edge buffers are contiguous");
        record(sec, bend.rest_angle.contiguous(), "bending rest_angle buffer is contiguous");
        record(sec, bend.color.contiguous(), "bending color buffer is contiguous");
        record(sec, isAligned(bend.e0.data, 64), "bending e0 buffer is 64-byte aligned");

        auto e0      = bend.e0.span();
        auto e1      = bend.e1.span();
        auto e2      = bend.e2.span();
        auto e3      = bend.e3.span();
        auto rest    = bend.rest_angle.span();
        auto stiff   = bend.stiffness.span();
        auto lambda  = bend.lambda.span();
        auto alpha   = bend.alpha.span();
        auto color   = bend.color.span();

        for (std::size_t idx = 0; idx < bend.m; ++idx) {
            e0[idx]     = static_cast<u32>(idx);
            e1[idx]     = static_cast<u32>(idx + 10);
            e2[idx]     = static_cast<u32>(idx + 20);
            e3[idx]     = static_cast<u32>(idx + 30);
            rest[idx]   = 0.1f * static_cast<float>(idx + 1);
            stiff[idx]  = 1.0f + static_cast<float>(idx);
            lambda[idx] = 0.05f * static_cast<float>(idx);
            alpha[idx]  = 0.02f * static_cast<float>(idx);
            color[idx]  = static_cast<u8>(idx + 1);
        }

        const auto bend_const = std::as_const(cloth).bending();
        auto e0_c             = bend_const.e0.span();
        auto rest_c           = bend_const.rest_angle.span();
        bool bending_ok       = true;
        for (std::size_t idx = 0; idx < bend.m; ++idx) {
            bending_ok = bending_ok && e0_c[idx] == static_cast<u32>(idx) && rest_c[idx] == 0.1f * static_cast<float>(idx + 1);
        }
        record(sec, bending_ok, "bending constraint data persists across const view");

        sections.push_back(std::move(sec));
    }

    // Triangular elastic material buffers
    {
        Section sec{"Tri-Elastic Columns"};
        ClothData cloth{};
        cloth.allocate_tri_elastic(2);
        auto tre = cloth.tri_elastic();
        record(sec, cloth.num_tri_elastic() == 2, "num_tri_elastic matches allocation");
        record(sec, tre.m == 2, "tri_elastic view exposes 2 entries");
        record(sec, tre.f0.contiguous() && tre.area.contiguous(), "tri_elastic columns are contiguous");
        record(sec, isAligned(tre.area.data, 64), "tri_elastic area buffer is 64-byte aligned");

        auto f0    = tre.f0.span();
        auto f1    = tre.f1.span();
        auto f2    = tre.f2.span();
        auto area  = tre.area.span();
        auto dm00  = tre.DmInv00.span();
        auto dm01  = tre.DmInv01.span();
        auto dm10  = tre.DmInv10.span();
        auto dm11  = tre.DmInv11.span();
        auto young = tre.youngs.span();
        auto pois  = tre.poisson.span();
        auto r00   = tre.R00.span();
        auto r01   = tre.R01.span();
        auto r10   = tre.R10.span();
        auto r11   = tre.R11.span();

        for (std::size_t idx = 0; idx < tre.m; ++idx) {
            f0[idx]    = static_cast<u32>(idx);
            f1[idx]    = static_cast<u32>(idx + 1u);
            f2[idx]    = static_cast<u32>(idx + 2u);
            area[idx]  = 0.25f * static_cast<float>(idx + 1);
            dm00[idx]  = 1.0f + static_cast<float>(idx);
            dm01[idx]  = 2.0f + static_cast<float>(idx);
            dm10[idx]  = 3.0f + static_cast<float>(idx);
            dm11[idx]  = 4.0f + static_cast<float>(idx);
            young[idx] = 100.0f + static_cast<float>(idx);
            pois[idx]  = 0.3f + 0.01f * static_cast<float>(idx);
            r00[idx]   = 1.0f;
            r01[idx]   = 0.0f;
            r10[idx]   = 0.0f;
            r11[idx]   = 1.0f;
        }

        const auto tre_const = std::as_const(cloth).tri_elastic();
        auto f0_c            = tre_const.f0.span();
        auto area_c          = tre_const.area.span();
        bool tre_ok          = true;
        for (std::size_t idx = 0; idx < tre.m; ++idx) {
            tre_ok = tre_ok && f0_c[idx] == static_cast<u32>(idx) && area_c[idx] == 0.25f * static_cast<float>(idx + 1);
        }
        record(sec, tre_ok, "tri_elastic data preserved across const view");

        sections.push_back(std::move(sec));
    }

    // Cross-field allocation should not interfere between categories.
    {
        Section sec{"Cross-Allocation"};
        ClothData cloth{};
        cloth.allocate_particles(3);
        cloth.allocate_distance(2);
        cloth.allocate_triangles(1);
        cloth.allocate_bending(1);
        cloth.allocate_tri_elastic(1);

        record(sec, cloth.num_particles() == 3, "particle count intact with mixed allocations");
        record(sec, cloth.num_edges() == 2, "edge count intact with mixed allocations");
        record(sec, cloth.num_faces() == 1, "face count intact with mixed allocations");
        record(sec, cloth.num_bending() == 1, "bending count intact with mixed allocations");
        record(sec, cloth.num_tri_elastic() == 1, "tri_elastic count intact with mixed allocations");

        auto particles = cloth.particles();
        auto dist      = cloth.distance();
        record(sec, particles.px.data != dist.rest.data, "particle and distance storage are disjoint");

        sections.push_back(std::move(sec));
    }

    // Emit aggregate report
    bool overall_ok = true;
    std::cout << "================ ClothData Verification Report ================\n";
    for (const auto& section : sections) {
        bool section_ok = std::all_of(section.entries.begin(), section.entries.end(), [](const Entry& e) { return e.ok; });
        overall_ok = overall_ok && section_ok;
        std::cout << "Section: " << section.name << " -> " << (section_ok ? "PASS" : "FAIL") << '\n';
        for (const auto& entry : section.entries) {
            std::cout << "  [" << mark(entry.ok) << "] " << entry.message << '\n';
        }
        std::cout << '\n';
    }
    std::cout << "Overall result: " << (overall_ok ? "PASS" : "FAIL") << '\n';
    std::cout << "==============================================================\n";

    return overall_ok ? 0 : 1;
}

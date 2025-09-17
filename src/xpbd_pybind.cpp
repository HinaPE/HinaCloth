#include "xpbd.h"
#include "cloth_data.h"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace {

using namespace HinaPE;

struct ClothBlueprint {
    std::size_t width{0};
    std::size_t height{0};
    float spacing{0.02f};
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> pz;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vz;
    std::vector<float> inv_mass;
    std::vector<u8> pinned;
    std::vector<u32> edge_i;
    std::vector<u32> edge_j;
    std::vector<float> rest;
    std::vector<float> compliance;
    std::vector<float> lambda;
    std::vector<float> alpha;
    std::vector<u8> color;
};

ClothBlueprint make_grid_blueprint(std::size_t width, std::size_t height, float spacing) {
    if (width < 2 || height < 2) {
        throw std::invalid_argument("Grid dimensions must be >= 2");
    }
    if (!(spacing > 0.0f)) {
        throw std::invalid_argument("Grid spacing must be positive");
    }

    ClothBlueprint bp{};
    bp.width   = width;
    bp.height  = height;
    bp.spacing = spacing;

    const std::size_t particle_count = width * height;
    const std::size_t horizontal     = (width - 1) * height;
    const std::size_t vertical       = width * (height - 1);
    const std::size_t edge_count     = horizontal + vertical;

    bp.px.resize(particle_count);
    bp.py.resize(particle_count);
    bp.pz.assign(particle_count, 0.0f);
    bp.vx.assign(particle_count, 0.0f);
    bp.vy.assign(particle_count, 0.0f);
    bp.vz.assign(particle_count, 0.0f);
    bp.inv_mass.assign(particle_count, 1.0f);
    bp.pinned.assign(particle_count, 0);

    bp.edge_i.reserve(edge_count);
    bp.edge_j.reserve(edge_count);
    bp.rest.reserve(edge_count);
    bp.compliance.reserve(edge_count);
    bp.lambda.reserve(edge_count);
    bp.alpha.reserve(edge_count);
    bp.color.reserve(edge_count);

    const float diag_offset = spacing * 0.5f;

    for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
            const std::size_t idx = y * width + x;
            bp.px[idx]            = static_cast<float>(x) * spacing;
            bp.py[idx]            = static_cast<float>(height - 1 - y) * spacing + ((y % 2) ? diag_offset : 0.0f);

            if (y == 0) {
                bp.pinned[idx]   = 1;
                bp.inv_mass[idx] = 0.0f;
            }
        }
    }

    auto append_edge = [&](std::uint32_t i, std::uint32_t j, std::uint8_t color) {
        bp.edge_i.push_back(i);
        bp.edge_j.push_back(j);
        bp.rest.push_back(spacing);
        bp.compliance.push_back(0.0f);
        bp.lambda.push_back(0.0f);
        bp.alpha.push_back(0.0f);
        bp.color.push_back(color);
    };

    for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t x = 0; x + 1 < width; ++x) {
            const std::uint32_t idx   = static_cast<std::uint32_t>(y * width + x);
            const std::uint32_t jdx   = static_cast<std::uint32_t>(y * width + x + 1);
            const std::uint8_t color  = static_cast<std::uint8_t>(x % 2);
            append_edge(idx, jdx, color);
        }
    }
    for (std::size_t y = 0; y + 1 < height; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
            const std::uint32_t idx   = static_cast<std::uint32_t>(y * width + x);
            const std::uint32_t jdx   = static_cast<std::uint32_t>((y + 1) * width + x);
            const std::uint8_t color  = static_cast<std::uint8_t>(2 + (y % 2));
            append_edge(idx, jdx, color);
        }
    }

    return bp;
}

void load_cloth_state(ClothData& cloth, const ClothBlueprint& bp) {
    cloth.allocate_particles(bp.px.size());
    cloth.allocate_distance(bp.edge_i.size());
    cloth.allocate_triangles(0);
    cloth.allocate_bending(0);
    cloth.allocate_tri_elastic(0);

    auto particles = cloth.particles();
    std::copy(bp.px.begin(), bp.px.end(), particles.px.span().begin());
    std::copy(bp.py.begin(), bp.py.end(), particles.py.span().begin());
    std::copy(bp.pz.begin(), bp.pz.end(), particles.pz.span().begin());
    std::copy(bp.vx.begin(), bp.vx.end(), particles.vx.span().begin());
    std::copy(bp.vy.begin(), bp.vy.end(), particles.vy.span().begin());
    std::copy(bp.vz.begin(), bp.vz.end(), particles.vz.span().begin());
    std::copy(bp.inv_mass.begin(), bp.inv_mass.end(), particles.inv_mass.span().begin());
    std::copy(bp.pinned.begin(), bp.pinned.end(), particles.pinned.span().begin());

    if (bp.edge_i.empty()) {
        return;
    }

    auto dist = cloth.distance();
    std::copy(bp.edge_i.begin(), bp.edge_i.end(), dist.i.span().begin());
    std::copy(bp.edge_j.begin(), bp.edge_j.end(), dist.j.span().begin());
    std::copy(bp.rest.begin(), bp.rest.end(), dist.rest.span().begin());
    std::copy(bp.compliance.begin(), bp.compliance.end(), dist.compliance.span().begin());
    std::copy(bp.lambda.begin(), bp.lambda.end(), dist.lambda.span().begin());
    std::copy(bp.alpha.begin(), bp.alpha.end(), dist.alpha.span().begin());
    std::copy(bp.color.begin(), bp.color.end(), dist.color.span().begin());
}

class XPBDSimulator {
public:
    XPBDSimulator(std::size_t width, std::size_t height, float spacing) : blueprint_(make_grid_blueprint(width, height, spacing)) {
        params_.time_step                 = 1.0f / 60.0f;
        params_.substeps                  = 4;
        params_.solver_iterations         = 8;
        params_.enable_distance_constraints = true;
        params_.enable_bending_constraints  = false;
        params_.velocity_damping          = 0.0f;
        reset();
    }

    void reset() {
        load_cloth_state(cloth_, blueprint_);
    }

    void set_time_step(float dt) {
        if (!(dt > 0.0f)) {
            throw std::invalid_argument("time_step must be positive");
        }
        params_.time_step = dt;
    }

    void set_substeps(int substeps) {
        if (substeps <= 0) {
            throw std::invalid_argument("substeps must be > 0");
        }
        params_.substeps = substeps;
    }

    void set_solver_iterations(int iterations) {
        if (iterations <= 0) {
            throw std::invalid_argument("solver_iterations must be > 0");
        }
        params_.solver_iterations = iterations;
    }

    void set_velocity_damping(float damping) {
        params_.velocity_damping = std::clamp(damping, 0.0f, 1.0f);
    }

    void set_gravity(const std::array<float, 3>& gravity) {
        params_.gravity = gravity;
    }

    void enable_distance_constraints(bool enabled) {
        params_.enable_distance_constraints = enabled;
    }

    std::size_t particle_count() const {
        return cloth_.num_particles();
    }

    std::size_t edge_count() const {
        return cloth_.num_edges();
    }

    void step_native(std::size_t steps) {
        run_steps(xpbd_step_native, steps);
    }

    void step_tbb(std::size_t steps) {
        run_steps(xpbd_step_tbb, steps);
    }

    void step_avx2(std::size_t steps) {
        run_steps(xpbd_step_avx2, steps);
    }

    py::array_t<float> positions() const {
        const auto particles = cloth_.particles();
        const auto count     = particles.n;
        py::array_t<float> array({static_cast<py::ssize_t>(count), static_cast<py::ssize_t>(3)});
        auto accessor = array.mutable_unchecked<2>();
        const auto px = particles.px.span();
        const auto py_span = particles.py.span();
        const auto pz = particles.pz.span();
        for (std::size_t i = 0; i < count; ++i) {
            accessor(static_cast<py::ssize_t>(i), 0) = px[i];
            accessor(static_cast<py::ssize_t>(i), 1) = py_span[i];
            accessor(static_cast<py::ssize_t>(i), 2) = pz[i];
        }
        return array;
    }

    py::array_t<float> velocities() const {
        const auto particles = cloth_.particles();
        const auto count     = particles.n;
        py::array_t<float> array({static_cast<py::ssize_t>(count), static_cast<py::ssize_t>(3)});
        auto accessor = array.mutable_unchecked<2>();
        const auto vx = particles.vx.span();
        const auto vy = particles.vy.span();
        const auto vz = particles.vz.span();
        for (std::size_t i = 0; i < count; ++i) {
            accessor(static_cast<py::ssize_t>(i), 0) = vx[i];
            accessor(static_cast<py::ssize_t>(i), 1) = vy[i];
            accessor(static_cast<py::ssize_t>(i), 2) = vz[i];
        }
        return array;
    }

private:
    using StepFn = void (*)(ClothData&, const XPBDParams&);

    void run_steps(StepFn fn, std::size_t steps) {
        if (steps == 0) {
            return;
        }
        for (std::size_t i = 0; i < steps; ++i) {
            fn(cloth_, params_);
        }
    }

    ClothBlueprint blueprint_;
    ClothData cloth_;
    XPBDParams params_{};
};

} // namespace

PYBIND11_MODULE(xpbd_core, m) {
    py::class_<XPBDSimulator>(m, "XPBDSimulator")
        .def(py::init<std::size_t, std::size_t, float>(),
             py::arg("width"), py::arg("height"), py::arg("spacing") = 0.025f)
        .def("reset", &XPBDSimulator::reset)
        .def("set_time_step", &XPBDSimulator::set_time_step, py::arg("time_step"))
        .def("set_substeps", &XPBDSimulator::set_substeps, py::arg("substeps"))
        .def("set_solver_iterations", &XPBDSimulator::set_solver_iterations, py::arg("iterations"))
        .def("set_velocity_damping", &XPBDSimulator::set_velocity_damping, py::arg("damping"))
        .def("set_gravity", &XPBDSimulator::set_gravity, py::arg("gravity"))
        .def("enable_distance_constraints", &XPBDSimulator::enable_distance_constraints, py::arg("enabled"))
        .def("particle_count", &XPBDSimulator::particle_count)
        .def("edge_count", &XPBDSimulator::edge_count)
        .def("step_native", &XPBDSimulator::step_native, py::arg("steps") = 1)
        .def("step_tbb", &XPBDSimulator::step_tbb, py::arg("steps") = 1)
        .def("step_avx2", &XPBDSimulator::step_avx2, py::arg("steps") = 1)
        .def("positions", &XPBDSimulator::positions)
        .def("velocities", &XPBDSimulator::velocities);
}

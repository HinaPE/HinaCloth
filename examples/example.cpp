#include "api/sim.h"
#include <vector>

struct Float3 {
    float x, y, z;
};

int main() {
    std::vector<Float3> positions  = {{0, 0, 0}, {1, 0, 0}};
    std::vector<uint32_t> edges    = {0, 1};
    sim::FieldView fields[1]       = {{"position", sim::FieldType::F32, positions.data(), positions.size(), 3, sizeof(Float3)}};
    sim::RelationView relations[1] = {{edges.data(), 2, 1, "edges"}};

    auto r = sim::create(sim::BuildDesc{
        .state = sim::StateInit{.fields = fields, .field_count = 1},
        .topo  = sim::TopologyIn{.node_count = static_cast<uint32_t>(positions.size()), .relations = relations, .relation_count = 1},
        .policy =
            sim::Policy{
                .exec  = sim::PolicyExec{.layout = sim::DataLayout::Auto, .backend = sim::Backend::Auto, .threads = -1, .deterministic = false, .telemetry = true},
                .solve = sim::PolicySolve{.substeps = 1, .iterations = 8, .damping = 0.0f, .stepper = sim::TimeStepper::Symplectic},
            },
        .validate = sim::ValidateLevel::Strict,
    });
    if (r.status != sim::Status::Ok) return -1;
    sim::Solver* s = r.value;

    sim::step(s, 1.0f / 60.0f);

    sim::TelemetryFrame tf{};
    (void) sim::telemetry_query_frame(s, &tf);

    std::vector<float> out(positions.size() * 3);
    size_t count = 0;
    sim::copy_positions(s, out.data(), 0, &count);
    printf("count = %zu\n", count);
    return 0;
}

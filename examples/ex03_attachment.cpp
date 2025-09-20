#include "api/sim.h"
#include <vector>
#include <cstdint>
#include <cstdio>

using namespace sim;

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx){ return j*nx + i; }
static void make_grid(uint32_t nx, uint32_t ny, float dx,
                      std::vector<float>& pos,
                      std::vector<float>& vel,
                      std::vector<uint32_t>& edges){
    uint32_t n = nx*ny; pos.resize(3u*n); vel.assign(3u*n, 0.0f);
    for(uint32_t j=0;j<ny;++j){ for(uint32_t i=0;i<nx;++i){ uint32_t id=vid(i,j,nx);
        pos[3u*id+0]=(float)i*dx; pos[3u*id+1]=0.6f; pos[3u*id+2]=(float)j*dx; }}
    edges.clear();
    for(uint32_t j=0;j<ny;++j){ for(uint32_t i=0;i+1<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i+1,j,nx); edges.push_back(a); edges.push_back(b);} }
    for(uint32_t j=0;j+1<ny;++j){ for(uint32_t i=0;i<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i,j+1,nx); edges.push_back(a); edges.push_back(b);} }
}

struct RegionPayload { const char* name; uint32_t start; uint32_t count; float v[3]; };

int main(){
    uint32_t nx=24, ny=16; float dx=0.05f; std::vector<float> pos, vel; std::vector<uint32_t> edges; make_grid(nx,ny,dx,pos,vel,edges);
    FieldView fpos{"position", FieldType::F32, pos.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fields[2] = {fpos, fvel}; StateInit st{fields, 2};
    RelationView rel{edges.data(), 2, edges.size()/2, "edges"}; TopologyIn topo{nx*ny, &rel, 1};
    const char* tags[] = {"edges"}; FieldUse uses[] = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true}; OperatorsDecl ops{&op, 1};
    Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {2, 12, 0.02f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0}; Parameters params{nullptr, 0}; EventsScript ev{nullptr, 0};
    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
    auto r = create(bd); if (r.status!=Status::Ok){ std::printf("ex03: create failed\n"); return 1; }
    Solver* s = r.value;

    // Enable attachment
    const char* att = "attachment"; Command c_en{CommandTag::EnableOperator, &att, sizeof(att)}; push_command(s, c_en);
    // Pin top row
    RegionPayload pinTop{"inv_mass", 0u, nx, {0.0f,0,0}}; Command c_pin{CommandTag::SetFieldRegion, &pinTop, sizeof(pinTop)}; push_command(s, c_pin);
    // Set a vertical stripe at mid column with weight=0.7 and target shifted +X
    uint32_t mid = nx/2; for(uint32_t j=0;j<ny;++j){ uint32_t id = vid(mid,j,nx);
        RegionPayload w{"attach_w", id, 1u, {0.7f,0,0}}; Command cw{CommandTag::SetFieldRegion, &w, sizeof(w)}; push_command(s, cw);
        float tx = pos[3u*id+0] + 0.15f, ty = pos[3u*id+1], tz = pos[3u*id+2];
        RegionPayload t{"attach_target", id, 1u, {tx,ty,tz}}; Command ct{CommandTag::SetFieldRegion, &t, sizeof(t)}; push_command(s, ct);
    }
    flush_commands(s, ApplyPhase::BeforeFrame);

    const float dt = 1.0f/60.0f; for(int f=0; f<150; ++f){ step(s, dt); }
    TelemetryFrame tf{}; Status st_tf = telemetry_query_frame(s, &tf); (void)st_tf;
    std::printf("ex03: step_ms=%.3f residual=%.6f\n", tf.step_ms, tf.residual_avg);
    destroy(s);
    return 0;
}

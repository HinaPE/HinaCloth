#include "api/sim.h"
#include <vector>
#include <cstdint>
#include <cstdio>

using namespace sim;

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx){ return j*nx + i; }
static void make_grid(uint32_t nx, uint32_t ny, float dx,
                      std::vector<float>& pos,
                      std::vector<float>& vel,
                      std::vector<uint32_t>& edges,
                      std::vector<uint32_t>& bend_pairs){
    uint32_t n = nx*ny; pos.resize(3u*n); vel.assign(3u*n, 0.0f);
    for(uint32_t j=0;j<ny;++j){ for(uint32_t i=0;i<nx;++i){ uint32_t id=vid(i,j,nx);
        pos[3u*id+0]=(float)i*dx; pos[3u*id+1]=0.5f; pos[3u*id+2]=(float)j*dx; }}
    edges.clear();
    for(uint32_t j=0;j<ny;++j){ for(uint32_t i=0;i+1<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i+1,j,nx); edges.push_back(a); edges.push_back(b);} }
    for(uint32_t j=0;j+1<ny;++j){ for(uint32_t i=0;i<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i,j+1,nx); edges.push_back(a); edges.push_back(b);} }
    bend_pairs.clear();
    for(uint32_t j=0;j+1<ny;++j){ for(uint32_t i=0;i+1<nx;++i){
        uint32_t A=vid(i,j,nx), B=vid(i+1,j,nx), C=vid(i,j+1,nx), D=vid(i+1,j+1,nx);
        bend_pairs.push_back(C); bend_pairs.push_back(B); bend_pairs.push_back(A); bend_pairs.push_back(D);
    }}
}

int main(){
    uint32_t nx=24, ny=16; float dx=0.05f; std::vector<float> pos, vel; std::vector<uint32_t> edges, quads; make_grid(nx,ny,dx,pos,vel,edges,quads);
    FieldView fpos{"position", FieldType::F32, pos.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fields[2] = {fpos, fvel}; StateInit st{fields, 2};
    RelationView rel_edges{edges.data(), 2, edges.size()/2, "edges"};
    RelationView rel_bend{quads.data(), 4, quads.size()/4, "bend_pairs"};
    RelationView rels[2] = {rel_edges, rel_bend}; TopologyIn topo{nx*ny, rels, 2};
    const char* tagsd[] = {"edges"}; FieldUse uses[] = {{"position", true}};
    OperatorDecl opd{"distance", tagsd, 1, uses, 1, OpStage::Solve, true};
    const char* tagsb[] = {"bend_pairs"}; OperatorDecl opb{"bending", tagsb, 1, uses, 1, OpStage::Solve, true};
    OperatorDecl ops_arr[2] = {opd, opb}; OperatorsDecl ops{ops_arr, 2};
    Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {2, 10, 0.02f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0}; Parameters params{nullptr, 0}; EventsScript ev{nullptr, 0};
    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
    auto r = create(bd); if (r.status!=Status::Ok){ std::printf("ex04: create failed\n"); return 1; }
    Solver* s = r.value;

    const char* bend = "bending"; Command c_en{CommandTag::EnableOperator, &bend, sizeof(bend)}; push_command(s, c_en);
    flush_commands(s, ApplyPhase::BeforeFrame);

    const float dt = 1.0f/60.0f; for(int f=0; f<120; ++f) step(s, dt);
    TelemetryFrame tf{}; Status st_tf = telemetry_query_frame(s, &tf); (void)st_tf;
    std::printf("ex04: step_ms=%.3f residual=%.6f\n", tf.step_ms, tf.residual_avg);
    destroy(s);
    return 0;
}

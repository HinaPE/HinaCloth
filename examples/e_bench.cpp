#include "api/sim.h"
#include "api/capability.h"
#include <vector>
#include <string>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <algorithm>

using namespace sim;

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx) { return j * nx + i; }

static void make_grid(uint32_t nx, uint32_t ny, float dx,
                      std::vector<float>& pos,
                      std::vector<float>& vel,
                      std::vector<uint32_t>& edges) {
    uint32_t n = nx * ny;
    pos.resize(3u * n);
    vel.assign(3u * n, 0.0f);
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t id = vid(i,j,nx);
            pos[3u*id+0] = (float) i * dx;
            pos[3u*id+1] = 0.5f;
            pos[3u*id+2] = (float) j * dx;
        }
    }
    edges.clear();
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = vid(i,j,nx), b = vid(i+1,j,nx);
            edges.push_back(a); edges.push_back(b);
        }
    }
    for (uint32_t j = 0; j + 1 < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t a = vid(i,j,nx), b = vid(i,j+1,nx);
            edges.push_back(a); edges.push_back(b);
        }
    }
}

static Backend parse_backend(const char* s) {
    if (!s) return Backend::Auto;
    if (!std::strcmp(s, "native")) return Backend::Native;
    if (!std::strcmp(s, "avx2"))   return Backend::AVX2;
    if (!std::strcmp(s, "tbb"))    return Backend::TBB;
    return Backend::Auto;
}
static DataLayout parse_layout(const char* s) {
    if (!s) return DataLayout::Auto;
    if (!std::strcmp(s, "soa"))     return DataLayout::SoA;
    if (!std::strcmp(s, "blocked")) return DataLayout::Blocked;
    return DataLayout::Auto;
}

struct Args {
    Backend backend = Backend::Auto;
    DataLayout layout = DataLayout::Auto;
    int threads = -1;
    uint32_t nx = 64, ny = 64;
    int frames = 120;
    int substeps = 2;
    int iterations = 10;
    bool sweep = false;
};

static Args parse_args(int ac, char** av) {
    Args a{};
    for (int i=1;i<ac;++i){
        if (std::strncmp(av[i], "--backend=", 10)==0) a.backend = parse_backend(av[i]+10);
        else if (std::strncmp(av[i], "--layout=", 9)==0) a.layout = parse_layout(av[i]+9);
        else if (std::strncmp(av[i], "--threads=", 10)==0) a.threads = std::atoi(av[i]+10);
        else if (std::strncmp(av[i], "--nx=", 5)==0) a.nx = (uint32_t) std::atoi(av[i]+5);
        else if (std::strncmp(av[i], "--ny=", 5)==0) a.ny = (uint32_t) std::atoi(av[i]+5);
        else if (std::strncmp(av[i], "--frames=", 9)==0) a.frames = std::atoi(av[i]+9);
        else if (std::strncmp(av[i], "--iters=", 8)==0) a.iterations = std::atoi(av[i]+8);
        else if (std::strncmp(av[i], "--subs=", 7)==0) a.substeps = std::atoi(av[i]+7);
        else if (std::strcmp(av[i], "--sweep")==0) a.sweep = true;
    }
    return a;
}

static const char* backend_name(Backend b){
    switch(b){case Backend::Native:return "native";case Backend::AVX2:return "avx2";case Backend::TBB:return "tbb";default:return "auto";}
}
static const char* layout_name(DataLayout l){
    switch(l){case DataLayout::SoA:return "soa";case DataLayout::Blocked:return "blocked";default:return "auto";}
}

static double run_case(uint32_t nx, uint32_t ny, Backend b, DataLayout l, int threads, int frames, int subs, int iters) {
    std::vector<float> pos, vel; std::vector<uint32_t> edges;
    make_grid(nx, ny, 0.05f, pos, vel, edges);
    FieldView fpos{"position", FieldType::F32, pos.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx*ny, 3, sizeof(float)*3};
    FieldView flds[2] = {fpos,fvel}; StateInit st{flds,2};
    RelationView rel{edges.data(), 2, edges.size()/2, "edges"}; TopologyIn topo{nx*ny,&rel,1};
    // Operators: distance only
    const char* tags[] = {"edges"}; FieldUse uses[] = {{"position", true}};
    OperatorDecl op{ "distance", tags, 1, uses, 1, OpStage::Solve, true}; OperatorsDecl ops{ &op, 1};
    Parameters params{nullptr,0}; SpaceDesc sp{SpaceType::Lagrangian, 1, 0};
    Policy pol{{l,b,threads,true,true},{subs,iters,0.02f,TimeStepper::Symplectic}};
    BuildDesc bd{st, params, topo, pol, sp, ops, EventsScript{nullptr,0}, ValidateLevel::Strict, {true, 64}};
    auto r = create(bd); if (r.status!=Status::Ok) return -1.0;
    Solver* s = r.value;
    const float dt = 1.0f/60.0f;
    double sum_ms = 0.0; int ok_frames=0;
    for(int f=0; f<frames; ++f){ step(s, dt); TelemetryFrame tf{}; telemetry_query_frame(s, &tf); if (tf.step_ms>0){sum_ms += tf.step_ms; ok_frames++;} }
    destroy(s);
    return ok_frames>0 ? (sum_ms/ok_frames) : -1.0;
}

int main(int ac, char** av){
    Args a = parse_args(ac,av);
    std::printf("backend,layout,threads,nx,ny,edges,substeps,iterations,avg_ms\n");
    if (!a.sweep) {
        double ms = run_case(a.nx,a.ny,a.backend,a.layout,a.threads,a.frames,a.substeps,a.iterations);
        uint64_t edges = (uint64_t)( (a.nx*(uint64_t)a.ny - a.ny) + (a.nx*(uint64_t)a.ny - a.nx) );
        std::printf("%s,%s,%d,%u,%u,%llu,%d,%d,%.4f\n", backend_name(a.backend), layout_name(a.layout), a.threads, a.nx, a.ny, (unsigned long long)edges, a.substeps, a.iterations, ms);
        return 0;
    }
    // Sweep a few sizes and backends available
    std::vector<Capability> caps(enumerate_capabilities(nullptr,0));
    caps.resize(enumerate_capabilities(caps.data(), caps.size()));
    std::vector<std::pair<uint32_t,uint32_t>> sizes = {{32,32},{64,64},{96,96}};
    for (auto& c : caps){
        Backend b = c.backend; DataLayout l = c.layout;
        for (auto [nx,ny] : sizes){
            double ms = run_case(nx,ny,b,l,a.threads,a.frames,a.substeps,a.iterations);
            uint64_t edges = (uint64_t)( (nx*(uint64_t)ny - ny) + (nx*(uint64_t)ny - nx) );
            std::printf("%s,%s,%d,%u,%u,%llu,%d,%d,%.4f\n", backend_name(b), layout_name(l), a.threads, nx, ny, (unsigned long long)edges, a.substeps, a.iterations, ms);
        }
    }
    return 0;
}


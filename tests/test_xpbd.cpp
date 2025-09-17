#include "xpbd.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <limits>
#include <iomanip>
#include <string>

using namespace HinaPE;

// 简化的蓝图结构（只保留必要字段）
struct ClothBlueprint {
    std::size_t width{};
    std::size_t height{};
    float spacing{0.02f};
    std::vector<float> px, py, pz;
    std::vector<float> vx, vy, vz;
    std::vector<float> inv_mass;
    std::vector<uint8_t> pinned;
    std::vector<uint32_t> edge_i, edge_j;
    std::vector<float> rest, compliance, lambda, alpha;
};

static ClothBlueprint makeGrid(std::size_t w, std::size_t h, float spacing, float compliance_value) {
    ClothBlueprint bp; bp.width=w; bp.height=h; bp.spacing=spacing;
    const size_t n = w*h;
    bp.px.resize(n); bp.py.resize(n); bp.pz.resize(n,0.f);
    bp.vx.resize(n,0.f); bp.vy.resize(n,0.f); bp.vz.resize(n,0.f);
    bp.inv_mass.resize(n,1.f); bp.pinned.resize(n,0);

    for(size_t y=0;y<h;++y){
        for(size_t x=0;x<w;++x){
            size_t id = y*w+x;
            bp.px[id] = float(x)*spacing;
            bp.py[id] = float(h-1-y)*spacing;
            if(y==0){ bp.pinned[id]=1; bp.inv_mass[id]=0.f; }
        }
    }

    // 生成边（4-color 简化：横向 color= x&1, 纵向 color=2+(y&1) 可省略不存）
    auto add_edge=[&](uint32_t i,uint32_t j){ bp.edge_i.push_back(i); bp.edge_j.push_back(j); bp.rest.push_back(spacing); bp.compliance.push_back(compliance_value); bp.lambda.push_back(0.f); bp.alpha.push_back(0.f); };
    for(size_t y=0;y<h;++y) for(size_t x=0;x+1<w;++x) add_edge(uint32_t(y*w+x), uint32_t(y*w+x+1));
    for(size_t y=0;y+1<h;++y) for(size_t x=0;x<w;++x) add_edge(uint32_t(y*w+x), uint32_t((y+1)*w+x));

    // 初始扰动：对非 pinned 点扩大 10% 形成拉伸，使初始残差>0
    for(size_t i=0;i<n;++i){ if(!bp.pinned[i]){ bp.px[i]*=1.1f; bp.py[i]*=1.1f; } }

    return bp;
}

static void loadCloth(ClothData& cloth, const ClothBlueprint& bp){
    cloth.allocate_particles(bp.px.size());
    cloth.allocate_distance(bp.edge_i.size());
    cloth.allocate_triangles(0); cloth.allocate_bending(0); cloth.allocate_tri_elastic(0);
    auto p = cloth.particles();
    std::copy(bp.px.begin(), bp.px.end(), p.px.span().begin());
    std::copy(bp.py.begin(), bp.py.end(), p.py.span().begin());
    std::copy(bp.pz.begin(), bp.pz.end(), p.pz.span().begin());
    std::copy(bp.vx.begin(), bp.vx.end(), p.vx.span().begin());
    std::copy(bp.vy.begin(), bp.vy.end(), p.vy.span().begin());
    std::copy(bp.vz.begin(), bp.vz.end(), p.vz.span().begin());
    std::copy(bp.inv_mass.begin(), bp.inv_mass.end(), p.inv_mass.span().begin());
    std::copy(bp.pinned.begin(), bp.pinned.end(), p.pinned.span().begin());
    if(bp.edge_i.empty()) return;
    auto d = cloth.distance();
    std::copy(bp.edge_i.begin(), bp.edge_i.end(), d.i.span().begin());
    std::copy(bp.edge_j.begin(), bp.edge_j.end(), d.j.span().begin());
    std::copy(bp.rest.begin(), bp.rest.end(), d.rest.span().begin());
    std::copy(bp.compliance.begin(), bp.compliance.end(), d.compliance.span().begin());
    std::copy(bp.lambda.begin(), bp.lambda.end(), d.lambda.span().begin());
    std::copy(bp.alpha.begin(), bp.alpha.end(), d.alpha.span().begin());
}

struct ResidualMetrics { double mean_abs{0.0}; double rms{0.0}; double max_abs{0.0}; };

static ResidualMetrics computeResidual(const ClothData& cloth, const ClothBlueprint& bp){
    ResidualMetrics m; if(bp.edge_i.empty()) return m;
    auto pview = cloth.particles(); // const view
    // 直接使用 data 指针避免 span() 在 const T 上的静态断言
    const float* px = pview.px.data;
    const float* py = pview.py.data;
    const float* pz = pview.pz.data;
    double sum_abs=0.0,sum_sq=0.0,max_abs=0.0; size_t cnt=bp.edge_i.size();
    for(size_t c=0;c<cnt;++c){
        uint32_t i=bp.edge_i[c], j=bp.edge_j[c]; double rest=bp.rest[c];
        double dx=px[i]-px[j], dy=py[i]-py[j], dz=pz[i]-pz[j];
        double len=std::sqrt(dx*dx+dy*dy+dz*dz); double r=len-rest; double a=std::fabs(r);
        sum_abs+=a; sum_sq+=r*r; if(a>max_abs) max_abs=a; }
    double inv=1.0/double(cnt); m.mean_abs=sum_abs*inv; m.rms=std::sqrt(sum_sq*inv); m.max_abs=max_abs; return m;
}

static void run_case(float compliance_value){
    const std::size_t W=32, H=32; const float spacing=0.03f; // 小尺寸快速
    ClothBlueprint bp = makeGrid(W,H,spacing,compliance_value);
    ClothData cloth; loadCloth(cloth, bp);

    XPBDParams params; params.time_step=1.f/120.f; params.substeps=4; params.solver_iterations=10; params.enable_distance_constraints=true; params.velocity_damping=0.01f;

    const int total_steps = 200; // 模拟帧数

    std::vector<double> rms_hist; rms_hist.reserve(total_steps);

    for(int step=0; step<total_steps; ++step){
        xpbd_step_native(cloth, params); // 注意：当前实现未真正使用 compliance
        auto res = computeResidual(cloth, bp);
        rms_hist.push_back(res.rms);
        if(step==0 || (step+1)%20==0){
            std::cout << "  step=" << std::setw(3) << (step+1)
                      << " rms=" << std::scientific << std::setprecision(3) << res.rms
                      << " mean=" << res.mean_abs
                      << " max=" << res.max_abs << '\n';
        }
    }

    double first = rms_hist.front(); double last = rms_hist.back();
    std::cout << "  initial_rms=" << std::scientific << std::setprecision(3) << first
              << " final_rms=" << last
              << " ratio(final/initial)=" << (last/(first+1e-30)) << '\n';

    if(last < 1e-4) std::cout << "  => 判定: 收敛\n";
    else if(last > first*1.2) std::cout << "  => 判定: 发散/未收敛 (残差增大)\n";
    else std::cout << "  => 判定: 仍有较大残差 (可能缓慢收敛或受限)\n";

    std::cout << '\n';
}

int main(){
    std::cout << "XPBD 收敛性与 compliance 实验 (当前 solver 实现检查)\n";
    std::cout << "当前实现已支持 compliance (alpha_tilde = compliance * dt^-2).\n";

    std::vector<float> compliances = {0.0f, 1e-7f, 1e-5f, 1e-4f, 1e-3f, 1e-2f};

    for(float c: compliances){
        std::cout << "==== compliance = " << std::scientific << std::setprecision(1) << c << " ====\n";
        run_case(c);
    }

    std::cout << "提示: 当 compliance 增大, 约束变软, 残差允许更大; 可增加 solver_iterations 或减小 dt 以提升收敛。\n";
    return 0;
}

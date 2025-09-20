#include "vk_engine.h"
#include "api/sim.h"
#include <imgui.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace {
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
}

class SimOverviewRenderer final : public IRenderer {
public:
    void get_capabilities(const EngineContext&, RendererCaps& caps) override {
        caps = RendererCaps{};
        caps.presentation_mode          = PresentationMode::EngineBlit;
        caps.preferred_swapchain_format = VK_FORMAT_B8G8R8A8_UNORM;
        caps.color_attachments = { AttachmentRequest{ .name = "color", .format = VK_FORMAT_B8G8R8A8_UNORM, .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, .samples = VK_SAMPLE_COUNT_1_BIT, .aspect = VK_IMAGE_ASPECT_COLOR_BIT, .initial_layout = VK_IMAGE_LAYOUT_GENERAL } };
        caps.presentation_attachment = "color";
        caps.enable_imgui = true;
    }

    void initialize(const EngineContext&, const RendererCaps&, const FrameContext&) override {
        // Build a small cloth and create solver
        uint32_t nx=24, ny=16; float dx=0.05f;
        pos_.clear(); vel_.clear(); edges_.clear(); make_grid(nx,ny,dx,pos_,vel_,edges_);
        FieldView fpos{"position", FieldType::F32, pos_.data(), nx*ny, 3, sizeof(float)*3};
        FieldView fvel{"velocity", FieldType::F32, vel_.data(), nx*ny, 3, sizeof(float)*3};
        FieldView fields[2] = {fpos,fvel}; StateInit st{fields,2};
        RelationView rel{edges_.data(), 2, edges_.size()/2, "edges"}; TopologyIn topo{nx*ny, &rel, 1};
        const char* tags[] = {"edges"}; FieldUse uses[] = {{"position", true}};
        OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true}; OperatorsDecl ops{&op, 1};
        Param gy{"gravity_y", ParamType::F32, {.f32=-9.8f}}; Parameters params{&gy, 1};
        Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {2, 10, 0.02f, TimeStepper::Symplectic}};
        SpaceDesc sp{SpaceType::Lagrangian, 1, 0}; EventsScript ev{nullptr, 0};
        BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
        auto r = create(bd);
        solver_ = (r.status==Status::Ok) ? r.value : nullptr;
    }

    void destroy(const EngineContext&, const RendererCaps&) override {
        if (solver_) { sim::destroy(solver_); solver_ = nullptr; }
    }

    void simulate(const EngineContext&, const FrameContext& frm) override {
        if (!solver_) return;
        float dt = (float) (frm.dt_sec > 0.0 ? frm.dt_sec : 1.0/60.0);
        step(solver_, dt);
    }

    void on_imgui(const EngineContext&, const FrameContext&) override {
        if (!solver_) return;
        TelemetryFrame tf{}; (void)telemetry_query_frame(solver_, &tf);
        ImGui::Begin("HinaCloth Telemetry");
        ImGui::Text("step_ms: %.3f", tf.step_ms);
        ImGui::Text("residual_avg: %.6f", tf.residual_avg);
        ImGui::Text("substeps: %d", tf.solve_substeps);
        ImGui::Text("iterations: %d", tf.solve_iterations);
        ImGui::End();
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext&, const FrameContext& frm) override {
        if (frm.color_attachments.empty()) return;
        const AttachmentView& target = frm.color_attachments.front();
        auto barrier = [&](VkImageLayout oldL, VkImageLayout newL, VkPipelineStageFlags2 srcStage, VkPipelineStageFlags2 dstStage, VkAccessFlags2 srcAccess, VkAccessFlags2 dstAccess){
            VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
            b.srcStageMask = srcStage; b.dstStageMask = dstStage; b.srcAccessMask = srcAccess; b.dstAccessMask = dstAccess;
            b.oldLayout = oldL; b.newLayout = newL; b.image = target.image; b.subresourceRange = {target.aspect,0u,1u,0u,1u};
            VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO}; di.imageMemoryBarrierCount=1; di.pImageMemoryBarriers=&b; vkCmdPipelineBarrier2(cmd,&di);
        };
        barrier(VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_MEMORY_WRITE_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        // Clear background subtly
        float t = float(frm.time_sec);
        VkClearValue clear{.color={{0.06f + 0.02f*std::sinf(t*0.5f), 0.07f, 0.08f, 1.0f}}};
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color.imageView = target.view; color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; color.storeOp = VK_ATTACHMENT_STORE_OP_STORE; color.clearValue = clear;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea = {{0,0}, frm.extent}; ri.layerCount = 1; ri.colorAttachmentCount = 1; ri.pColorAttachments = &color;
        vkCmdBeginRendering(cmd, &ri);
        vkCmdEndRendering(cmd);

        barrier(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    }

private:
    sim::Solver* solver_{nullptr};
    std::vector<float> pos_, vel_;
    std::vector<uint32_t> edges_;
};

int main(){
    VulkanEngine engine;
    engine.configure_window(1280, 720, "HinaCloth Visualizer: Sim Overview");
    engine.set_renderer(std::make_unique<SimOverviewRenderer>());
    engine.init();
    engine.run();
    engine.cleanup();
    return 0;
}

#include "vk_engine.h"
#include "api/sim.h"
#include <imgui.h>
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

class SimControlsRenderer final : public IRenderer {
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
        // Create a 24x16 cloth
        nx_ = 24; ny_ = 16; float dx=0.05f;
        pos_.clear(); vel_.clear(); edges_.clear(); make_grid(nx_,ny_,dx,pos_,vel_,edges_);
        FieldView fpos{"position", FieldType::F32, pos_.data(), nx_*ny_, 3, sizeof(float)*3};
        FieldView fvel{"velocity", FieldType::F32, vel_.data(), nx_*ny_, 3, sizeof(float)*3};
        FieldView fields[2] = {fpos,fvel}; StateInit st{fields,2};
        RelationView rel{edges_.data(), 2, edges_.size()/2, "edges"}; TopologyIn topo{nx_*ny_, &rel, 1};
        const char* tags[] = {"edges"}; FieldUse uses[] = {{"position", true}};
        OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true}; OperatorsDecl ops{&op, 1};
        Param gy{"gravity_y", ParamType::F32, {.f32=-9.8f}}; Parameters params{&gy, 1};
        Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {1, 8, 0.01f, TimeStepper::Symplectic}};
        SpaceDesc sp{SpaceType::Lagrangian, 1, 0}; EventsScript ev{nullptr, 0};
        BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
        auto r = create(bd); solver_ = (r.status==Status::Ok) ? r.value : nullptr;
        // Initial chosen
        auto qc = query_chosen(solver_); if (qc.status==Status::Ok) chosen_ = qc.value;
        // Default UI states
        gravity_y_ = -9.8f; iters_ = 8; subs_ = 1; damping_ = 0.01f; pin_top_ = false; enable_attach_ = false; enable_bend_ = false;
    }

    void destroy(const EngineContext&, const RendererCaps&) override {
        if (solver_) { sim::destroy(solver_); solver_ = nullptr; }
    }

    void simulate(const EngineContext&, const FrameContext& frm) override {
        if (!solver_) return;
        // Apply queued commands at BeforeFrame
        if (dirty_) { (void)flush_commands(solver_, ApplyPhase::BeforeFrame); dirty_ = false; }
        float dt = (float) (frm.dt_sec > 0.0 ? frm.dt_sec : 1.0/60.0);
        step(solver_, dt);
    }

    void on_imgui(const EngineContext&, const FrameContext&) override {
        if (!solver_) return;
        ImGui::Begin("HinaCloth Controls");
        ImGui::Text("backend=%d layout=%d threads=%d", (int)chosen_.backend, (int)chosen_.layout, chosen_.threads);
        bool changed=false;
        changed |= ImGui::SliderFloat("gravity_y", &gravity_y_, -30.0f, 0.0f);
        changed |= ImGui::SliderInt("iterations", &iters_, 1, 64);
        changed |= ImGui::SliderInt("substeps", &subs_, 1, 8);
        changed |= ImGui::SliderFloat("damping", &damping_, 0.0f, 0.2f);
        if (ImGui::Checkbox("pin top row", &pin_top_)) changed = true;
        if (ImGui::Checkbox("enable attachment", &enable_attach_)) changed = true;
        if (ImGui::Checkbox("enable bending (demo)", &enable_bend_)) changed = true;
        if (changed) apply_controls();

        TelemetryFrame tf{}; (void)telemetry_query_frame(solver_, &tf);
        ImGui::Separator();
        ImGui::Text("step_ms: %.3f residual: %.6f", tf.step_ms, tf.residual_avg);
        ImGui::Text("sub=%d it=%d cmds=%llu rebuilds=%llu", tf.solve_substeps, tf.solve_iterations, (unsigned long long)tf.commands_applied, (unsigned long long)tf.structural_rebuilds);
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
        VkClearValue clear{.color={{0.05f,0.06f,0.07f,1.0f}}};
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
    void apply_controls(){
        // SetParam payloads
        struct ParamPayload { const char* name; float value; };
        ParamPayload pgy{"gravity_y", gravity_y_}; Command cgy{CommandTag::SetParam, &pgy, sizeof(pgy)}; push_command(solver_, cgy);
        ParamPayload pit{"iterations", (float)iters_}; Command cit{CommandTag::SetParam, &pit, sizeof(pit)}; push_command(solver_, cit);
        ParamPayload psu{"substeps", (float)subs_};   Command csu{CommandTag::SetParam, &psu, sizeof(psu)}; push_command(solver_, csu);
        ParamPayload pda{"damping", damping_};        Command cda{CommandTag::SetParam, &pda, sizeof(pda)}; push_command(solver_, cda);
        // Pin/unpin top row via SetFieldRegion
        struct RegionPayload { const char* name; uint32_t start; uint32_t count; float v[3]; };
        RegionPayload pinTop{"inv_mass", 0u, nx_, { pin_top_? 0.0f: 1.0f, 0,0}}; Command cpin{CommandTag::SetFieldRegion, &pinTop, sizeof(pinTop)}; push_command(solver_, cpin);
        // Enable/disable operators
        const char* att = "attachment"; const char* bend = "bending";
        if (enable_attach_) { Command cen{CommandTag::EnableOperator, &att, sizeof(att)}; push_command(solver_, cen);} else { Command cdis{CommandTag::DisableOperator, &att, sizeof(att)}; push_command(solver_, cdis);}
        if (enable_bend_)   { Command cen{CommandTag::EnableOperator, &bend, sizeof(bend)}; push_command(solver_, cen);} else { Command cdis{CommandTag::DisableOperator, &bend, sizeof(bend)}; push_command(solver_, cdis);}
        dirty_ = true;
    }

    sim::Solver* solver_{nullptr};
    Chosen chosen_{};
    uint32_t nx_{0}, ny_{0};
    std::vector<float> pos_, vel_;
    std::vector<uint32_t> edges_;
    // UI state
    float gravity_y_{-9.8f}; int iters_{8}; int subs_{1}; float damping_{0.01f}; bool pin_top_{false}; bool enable_attach_{false}; bool enable_bend_{false};
    bool dirty_{false};
};

int main(){
    VulkanEngine engine;
    engine.configure_window(1280, 720, "HinaCloth Visualizer: Sim Controls");
    engine.set_renderer(std::make_unique<SimControlsRenderer>());
    engine.init();
    engine.run();
    engine.cleanup();
    return 0;
}


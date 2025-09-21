#include "vk_engine.h"
#include "vv_camera.h"
#include <imgui.h>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "api/sim.h"
#include "api/build.h"
#include "api/state_in.h"
#include "api/topology_in.h"
#include "api/operators_in.h"
#include "api/parameters_in.h"

#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <fstream>

#ifndef VK_CHECK
#define VK_CHECK(x) do{VkResult r=(x); if(r!=VK_SUCCESS) throw std::runtime_error("Vulkan error: "+std::to_string(r)); }while(false)
#endif

static std::vector<char> load_spv(const std::string& p){ std::ifstream f(p, std::ios::binary | std::ios::ate); if(!f) throw std::runtime_error("open "+p); size_t s=(size_t)f.tellg(); f.seekg(0); std::vector<char> d(s); f.read(d.data(), (std::streamsize)s); return d; }
static VkShaderModule make_shader(VkDevice d, const std::vector<char>& b){ VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO}; ci.codeSize=(uint32_t)b.size(); ci.pCode=(const uint32_t*)b.data(); VkShaderModule m{}; VK_CHECK(vkCreateShaderModule(d,&ci,nullptr,&m)); return m; }

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx){ return j*nx + i; }

class HinaXPBDRenderer : public IRenderer {
public:
    void get_capabilities(const EngineContext&, RendererCaps& c) override {
        c = RendererCaps{}; c.enable_imgui = true; c.presentation_mode = PresentationMode::EngineBlit;
        c.color_attachments = { AttachmentRequest{ .name = "color", .format = VK_FORMAT_B8G8R8A8_UNORM } }; c.presentation_attachment = "color";
        c.depth_attachment = AttachmentRequest{ .name = "depth", .format = c.preferred_depth_format, .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, .samples = VK_SAMPLE_COUNT_1_BIT, .aspect = VK_IMAGE_ASPECT_DEPTH_BIT, .initial_layout = VK_IMAGE_LAYOUT_UNDEFINED };
        c.uses_depth = VK_TRUE;
    }

    void initialize(const EngineContext& e, const RendererCaps&, const FrameContext&) override {
        eng_ = e; dev_ = e.device; color_fmt_ = VK_FORMAT_B8G8R8A8_UNORM; depth_fmt_ = VK_FORMAT_D32_SFLOAT;
        build_sim_();
        build_gpu_buffers_(); build_pipelines_();
        cam_.set_mode(vv::CameraMode::Orbit); auto s = cam_.state(); s.target={0,0,0}; s.distance=2.0f; s.pitch_deg=20.0f; s.yaw_deg=-120.0f; s.znear=0.01f; s.zfar=200.0f; cam_.set_state(s); frame_scene_to_positions_();
        sim_accum_ = 0.0;
    }

    void destroy(const EngineContext& e, const RendererCaps&) override {
        (void)e; // silence unused parameter warning
        destroy_gpu_buffers_(); destroy_pipelines_(); if (solver_) sim::destroy(solver_); solver_ = nullptr; dev_ = VK_NULL_HANDLE; eng_ = {};
    }

    void update(const EngineContext&, const FrameContext& f) override {
        cam_.update(f.dt_sec, (int)f.extent.width, (int)f.extent.height); vp_w_=(int)f.extent.width; vp_h_=(int)f.extent.height;
        // simulation step
        if (params_.simulate && solver_) {
            sim_accum_ += f.dt_sec;
            double fixed = std::clamp<double>(params_.fixed_dt, 1.0/600.0, 1.0/30.0);
            int maxSteps = 4;
            while (sim_accum_ >= fixed && maxSteps--) {
                sim::step(solver_, (float)fixed);
                sim_accum_ -= fixed;
            }
        }
        // readback positions and upload
        if (!pos_buf_.mapped || !solver_) return;
        size_t outCount = 0; cpu_pos_.resize(3u*node_count_);
        sim::Status st = sim::copy_positions(solver_, cpu_pos_.data(), node_count_, &outCount);
        if (st == sim::Status::Ok && outCount >= node_count_) {
            std::memcpy(pos_buf_.mapped, cpu_pos_.data(), node_count_*sizeof(vv::float3));
        }
    }

    void on_event(const SDL_Event& e, const EngineContext& eng, const FrameContext* f) override {
        cam_.handle_event(e, &eng, f);
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext&, const FrameContext& f) override {
        if (f.color_attachments.empty() || !pipe_tri_.pipeline) return;
        const auto& color = f.color_attachments.front(); const auto* depth = f.depth_attachment;
        auto barrier_img = [&](VkImage img, VkImageAspectFlags aspect, VkImageLayout oldL, VkImageLayout newL, VkPipelineStageFlags2 src, VkPipelineStageFlags2 dst, VkAccessFlags2 sa, VkAccessFlags2 da){
            VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2}; b.srcStageMask=src; b.dstStageMask=dst; b.srcAccessMask=sa; b.dstAccessMask=da; b.oldLayout=oldL; b.newLayout=newL; b.image=img; b.subresourceRange={aspect,0,1,0,1}; VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO}; di.imageMemoryBarrierCount=1; di.pImageMemoryBarriers=&b; vkCmdPipelineBarrier2(cmd,&di);
        };
        barrier_img(color.image, color.aspect, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        if (depth) barrier_img(depth->image, depth->aspect, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT, 0, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
        VkClearValue clear_color{.color={{0.05f,0.06f,0.07f,1.0f}}}; VkClearValue clear_depth{.depthStencil={1.0f,0}};
        VkRenderingAttachmentInfo ca{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO}; ca.imageView=color.view; ca.imageLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; ca.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; ca.storeOp=VK_ATTACHMENT_STORE_OP_STORE; ca.clearValue=clear_color;
        VkRenderingAttachmentInfo da{}; if (depth){ da.sType=VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO; da.imageView=depth->view; da.imageLayout=VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL; da.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; da.storeOp=VK_ATTACHMENT_STORE_OP_STORE; da.clearValue=clear_depth; }
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea={{0,0}, f.extent}; ri.layerCount=1; ri.colorAttachmentCount=1; ri.pColorAttachments=&ca; ri.pDepthAttachment = depth? &da : nullptr; vkCmdBeginRendering(cmd,&ri);
        VkViewport vp{}; vp.x=0; vp.y=0; vp.width=(float)f.extent.width; vp.height=(float)f.extent.height; vp.minDepth=0; vp.maxDepth=1; VkRect2D sc{{0,0}, f.extent}; vkCmdSetViewport(cmd,0,1,&vp); vkCmdSetScissor(cmd,0,1,&sc);
        const vv::float4x4 V = cam_.view_matrix(); const vv::float4x4 P = cam_.proj_matrix(); vv::float4x4 MVP = vv::mul(P, V);
        struct PC { float mvp[16]; float color[4]; float pointSize; float _pad[3]; } pc{};
        std::memcpy(pc.mvp, MVP.m.data(), sizeof(pc.mvp));
        VkDeviceSize offs = 0; vkCmdBindVertexBuffers(cmd, 0, 1, &pos_buf_.buf, &offs);
        // Draw mesh (triangles)
        if (params_.show_mesh){
            pc.color[0]=0.55f; pc.color[1]=0.7f; pc.color[2]=0.95f; pc.color[3]=1.0f; pc.pointSize = params_.point_size;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_tri_.pipeline);
            vkCmdPushConstants(cmd, pipe_tri_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PC), &pc);
            vkCmdBindIndexBuffer(cmd, tri_idx_.buf, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, tri_count_, 1, 0, 0, 0);
        }
        // Draw constraints (lines)
        if (params_.show_constraints && line_idx_count_>0){
            pc.color[0]=0.9f; pc.color[1]=0.9f; pc.color[2]=0.9f; pc.color[3]=1.0f; pc.pointSize = params_.point_size;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_line_.pipeline);
            vkCmdPushConstants(cmd, pipe_line_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PC), &pc);
            vkCmdBindIndexBuffer(cmd, line_idx_.buf, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, line_idx_count_, 1, 0, 0, 0);
        }
        // Draw vertices (points)
        if (params_.show_vertices){
            pc.color[0]=1.0f; pc.color[1]=1.0f; pc.color[2]=1.0f; pc.color[3]=1.0f; pc.pointSize = params_.point_size;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_point_.pipeline);
            vkCmdPushConstants(cmd, pipe_point_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PC), &pc);
            vkCmdDraw(cmd, node_count_, 1, 0, 0);
        }
        vkCmdEndRendering(cmd);
        barrier_img(color.image, color.aspect, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT);
    }

    void on_imgui(const EngineContext& eng, const FrameContext&) override {
        auto* host = static_cast<vv_ui::TabsHost*>(eng.services);
        if (host) {
            host->add_overlay([this]{ cam_.imgui_draw_nav_overlay_space_tint(); });
            host->add_overlay([this]{ cam_.imgui_draw_mini_axis_gizmo(); });
        }
        if (!host) return;
        host->add_tab("HinaCloth XPBD", [this]{
            ImGui::Checkbox("Simulate", &params_.simulate); ImGui::SameLine(); if (ImGui::Button("Step")) { sim::step(solver_, std::clamp<float>((float)params_.fixed_dt, 1.f/600.f, 1.f/30.f)); }
            ImGui::SameLine(); if (ImGui::Button("Reset")) { reset_sim_(); frame_scene_to_positions_(); }
            ImGui::Separator();
            ImGui::Checkbox("Mesh", &params_.show_mesh); ImGui::SameLine();
            ImGui::Checkbox("Vertices", &params_.show_vertices); ImGui::SameLine();
            ImGui::Checkbox("Constraints", &params_.show_constraints);
            ImGui::SliderFloat("Point Size", &params_.point_size, 1.0f, 12.0f);
            ImGui::SliderFloat("Fixed dt (s)", &params_.fixed_dt, 1.0f/240.0f, 1.0f/30.0f, "%.4f");
            ImGui::Separator(); ImGui::InputInt("Grid X", &params_.grid_x); ImGui::SameLine(); ImGui::InputInt("Grid Y", &params_.grid_y); ImGui::SliderFloat("Spacing", &params_.spacing, 0.02f, 0.2f);
            if (ImGui::Button("Rebuild Grid")) { rebuild_grid_(params_.grid_x, params_.grid_y, params_.spacing); frame_scene_to_positions_(); }
            ImGui::SameLine(); if (ImGui::Button("Frame Cloth")) { frame_scene_to_positions_(); }
        });
        host->add_tab("Camera", [this]{ cam_.imgui_panel_contents(); });
    }

private:
    struct Params { bool simulate{true}; float fixed_dt{1.0f/120.0f}; bool show_mesh{true}; bool show_vertices{true}; bool show_constraints{true}; float point_size{5.0f}; int grid_x{20}, grid_y{20}; float spacing{0.06f}; } params_{};

    vv::CameraService cam_{}; EngineContext eng_{}; VkDevice dev_{VK_NULL_HANDLE}; VkFormat color_fmt_{VK_FORMAT_B8G8R8A8_UNORM}; VkFormat depth_fmt_{VK_FORMAT_D32_SFLOAT}; int vp_w_{0}, vp_h_{0};

    // HinaCloth sim
    sim::Solver* solver_{nullptr};
    uint32_t nx_{0}, ny_{0}; float dx_{0.06f}; uint32_t node_count_{0};
    std::vector<uint32_t> edges_; // store structural (and possibly others) for visualization

    // CPU positions for upload
    std::vector<float> cpu_pos_;

    // GPU buffers
    struct GpuBuffer { VkBuffer buf{}; VmaAllocation alloc{}; void* mapped{}; size_t size{}; };
    GpuBuffer pos_buf_{}; // vec3 positions
    GpuBuffer tri_idx_{}; uint32_t tri_count_{0};
    GpuBuffer line_idx_{}; uint32_t line_idx_count_{0};

    struct Pipeline { VkPipeline pipeline{}; VkPipelineLayout layout{}; };
    Pipeline pipe_tri_{}, pipe_line_{}, pipe_point_{};

    double sim_accum_{0.0};

private:
    void build_sim_(){
        // Default grid
        rebuild_grid_(params_.grid_x, params_.grid_y, params_.spacing);
    }

    void reset_sim_(){ rebuild_grid_(nx_, ny_, dx_); }

    void rebuild_grid_(uint32_t nx, uint32_t ny, float dx){
        // Cleanup GPU and solver
        if (solver_) { sim::destroy(solver_); solver_ = nullptr; }
        destroy_buffer_(tri_idx_); destroy_buffer_(line_idx_); // keep pos buffer; resize later if needed
        // Build initial state arrays
        nx_ = std::max(2u, nx); ny_ = std::max(2u, ny); dx_ = dx; node_count_ = nx_*ny_;
        std::vector<float> pos(3u*node_count_), vel(3u*node_count_, 0.0f);
        for(uint32_t j=0;j<ny_;++j){ for(uint32_t i=0;i<nx_;++i){ uint32_t id=vid(i,j,nx_); pos[3u*id+0] = (float)i*dx_; pos[3u*id+1] = 0.8f; pos[3u*id+2] = (float)j*dx_; }}
        // Relations: structural edges (H/V) + diagonals for nicer look
        edges_.clear(); edges_.reserve(nx_*ny_*4);
        for(uint32_t j=0;j<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ uint32_t a=vid(i,j,nx_), b=vid(i+1,j,nx_); edges_.push_back(a); edges_.push_back(b);} }
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i<nx_;++i){ uint32_t a=vid(i,j,nx_), b=vid(i,j+1,nx_); edges_.push_back(a); edges_.push_back(b);} }
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ uint32_t a=vid(i,j,nx_), b=vid(i+1,j+1,nx_); edges_.push_back(a); edges_.push_back(b); uint32_t c=vid(i+1,j,nx_), d=vid(i,j+1,nx_); edges_.push_back(c); edges_.push_back(d);} }
        // Build solver
        sim::FieldView fpos{"position", sim::FieldType::F32, pos.data(), node_count_, 3, sizeof(float)*3};
        sim::FieldView fvel{"velocity", sim::FieldType::F32, vel.data(), node_count_, 3, sizeof(float)*3};
        sim::FieldView flds[2] = {fpos, fvel}; sim::StateInit st{flds, 2};
        sim::RelationView rel{edges_.data(), 2, edges_.size()/2, "edges"}; sim::TopologyIn topo{node_count_, &rel, 1};
        const char* tags[] = {"edges"}; sim::FieldUse uses[] = {{"position", true}};
        sim::OperatorDecl op{ "distance", tags, 1, uses, 1, sim::OpStage::Solve, true}; sim::OperatorsDecl ops{ &op, 1};
        sim::Param gpy{"gravity_y", sim::ParamType::F32, {.f32=-9.8f}}; sim::Parameters params{&gpy, 1};
        sim::Policy pol{{sim::DataLayout::Auto, sim::Backend::Auto, -1, true, true},{1, 10, 0.0f, sim::TimeStepper::Symplectic}};
        sim::SpaceDesc sp{sim::SpaceType::Lagrangian, 1, 0}; sim::EventsScript ev{nullptr, 0};
        sim::BuildDesc bd{st, params, topo, pol, sp, ops, ev, sim::ValidateLevel::Strict, {true, 8}};
        auto r = sim::create(bd); if (r.status != sim::Status::Ok) throw std::runtime_error("HinaCloth create() failed");
        solver_ = r.value;
        // Pin top two vertices by zeroing inv_mass via SetFieldRegion
        struct RegionPayload { const char* name; uint32_t start; uint32_t count; float v[3]; };
        uint32_t id0 = vid(0, 0, nx_);
        uint32_t id1 = vid(nx_-1, 0, nx_);
        RegionPayload pinA{"inv_mass", id0, 1u, {0.0f, 0.0f, 0.0f}};
        RegionPayload pinB{"inv_mass", id1, 1u, {0.0f, 0.0f, 0.0f}};
        sim::Command cA{sim::CommandTag::SetFieldRegion, &pinA, sizeof(pinA)};
        sim::Command cB{sim::CommandTag::SetFieldRegion, &pinB, sizeof(pinB)};
        sim::push_command(solver_, cA);
        sim::push_command(solver_, cB);
        sim::flush_commands(solver_, sim::ApplyPhase::BeforeFrame);
        // Prepare GPU buffers for indices and positions
        rebuild_indices_();
        // Ensure pos buffer large enough
        if (!pos_buf_.buf || pos_buf_.size < node_count_*sizeof(vv::float3)) {
            destroy_buffer_(pos_buf_);
            create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, pos_buf_);
        }
        // Upload initial positions
        cpu_pos_.assign(pos.begin(), pos.end());
        if (pos_buf_.mapped) std::memcpy(pos_buf_.mapped, cpu_pos_.data(), node_count_*sizeof(vv::float3));
    }

    void rebuild_indices_(){
        // triangles
        std::vector<uint32_t> idx; idx.reserve((nx_-1)*(ny_-1)*6);
        auto id = [&](int x,int y){ return (uint32_t)(y*nx_ + x); };
        for(uint32_t y=0;y<ny_-1;++y){ for(uint32_t x=0;x<nx_-1;++x){ uint32_t a=id(x,y), b=id(x+1,y), c=id(x,y+1), d=id(x+1,y+1); idx.push_back(a); idx.push_back(b); idx.push_back(d); idx.push_back(a); idx.push_back(d); idx.push_back(c);} }
        tri_count_ = (uint32_t)idx.size();
        if (!tri_idx_.buf) create_buffer_(idx.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, tri_idx_);
        else if (tri_idx_.size < idx.size()*sizeof(uint32_t)) { destroy_buffer_(tri_idx_); create_buffer_(idx.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, tri_idx_); }
        if (!idx.empty()) std::memcpy(tri_idx_.mapped, idx.data(), idx.size()*sizeof(uint32_t));
        // lines from edges_
        line_idx_count_ = (uint32_t)edges_.size();
        if (!line_idx_.buf) create_buffer_(edges_.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_idx_);
        else if (line_idx_.size < edges_.size()*sizeof(uint32_t)) { destroy_buffer_(line_idx_); create_buffer_(edges_.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_idx_); }
        if (!edges_.empty()) std::memcpy(line_idx_.mapped, edges_.data(), edges_.size()*sizeof(uint32_t));
    }

    void frame_scene_to_positions_(){
        if (node_count_==0) return; vv::float3 mn{cpu_pos_[0], cpu_pos_[1], cpu_pos_[2]}, mx=mn; for(uint32_t i=0;i<node_count_;++i){ float x=cpu_pos_[3*i+0], y=cpu_pos_[3*i+1], z=cpu_pos_[3*i+2]; mn.x=std::min(mn.x,x); mn.y=std::min(mn.y,y); mn.z=std::min(mn.z,z); mx.x=std::max(mx.x,x); mx.y=std::max(mx.y,y); mx.z=std::max(mx.z,z);} mn.z-=0.2f; mx.z+=0.2f; cam_.set_scene_bounds(vv::BoundingBox{.min=mn,.max=mx,.valid=true}); cam_.frame_scene(1.12f);
    }

    // GPU helpers
    void create_buffer_(VkDeviceSize sz, VkBufferUsageFlags usage, VmaMemoryUsage memUsage, bool mapped, GpuBuffer& out){
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bi.size=sz; bi.usage=usage; bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo ai{}; ai.usage = memUsage; ai.flags = mapped? VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT : 0;
        VK_CHECK(vmaCreateBuffer(eng_.allocator, &bi, &ai, &out.buf, &out.alloc, nullptr)); out.size=(size_t)sz; out.mapped=nullptr; if (mapped) { vmaMapMemory(eng_.allocator, out.alloc, &out.mapped); }
    }
    void destroy_buffer_(GpuBuffer& b){ if (b.mapped) { vmaUnmapMemory(eng_.allocator, b.alloc); b.mapped=nullptr; } if (b.buf) vmaDestroyBuffer(eng_.allocator, b.buf, b.alloc); b = {}; }

    void build_gpu_buffers_(){ if (!pos_buf_.buf) create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, pos_buf_); rebuild_indices_(); }

    void build_pipelines_(){
        std::string dir(SHADER_OUTPUT_DIR); VkShaderModule vs = make_shader(dev_, load_spv(dir+"/cloth.vert.spv")); VkShaderModule fs = make_shader(dev_, load_spv(dir+"/cloth.frag.spv"));
        VkPipelineShaderStageCreateInfo st[2]{}; for(int i=0;i<2;++i) st[i].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; st[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st[0].module=vs; st[0].pName="main"; st[1]=st[0]; st[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module=fs;
        VkVertexInputBindingDescription bind{0, sizeof(vv::float3), VK_VERTEX_INPUT_RATE_VERTEX}; VkVertexInputAttributeDescription attr{0,0,VK_FORMAT_R32G32B32_SFLOAT,0};
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vi.vertexBindingDescriptionCount=1; vi.pVertexBindingDescriptions=&bind; vi.vertexAttributeDescriptionCount=1; vi.pVertexAttributeDescriptions=&attr;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth=1.0f;
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO}; ds.depthTestEnable=VK_TRUE; ds.depthWriteEnable=VK_TRUE; ds.depthCompareOp=VK_COMPARE_OP_LESS;
        VkPipelineColorBlendAttachmentState ba{}; ba.colorWriteMask=0xF; VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&ba;
        const VkDynamicState dyns[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR }; VkPipelineDynamicStateCreateInfo dsi{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; dsi.dynamicStateCount=2; dsi.pDynamicStates=dyns;
        VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT, 0, 96}; VkPipelineLayoutCreateInfo lci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; lci.pushConstantRangeCount=1; lci.pPushConstantRanges=&pcr; VK_CHECK(vkCreatePipelineLayout(dev_, &lci, nullptr, &pipe_tri_.layout)); pipe_line_.layout = pipe_tri_.layout; pipe_point_.layout = pipe_tri_.layout;
        VkPipelineRenderingCreateInfo rinfo{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; rinfo.colorAttachmentCount=1; rinfo.pColorAttachmentFormats=&color_fmt_; rinfo.depthAttachmentFormat=depth_fmt_;
        auto make_pipeline = [&](VkPrimitiveTopology topo, Pipeline& out){ VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=topo; VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO}; pci.pNext=&rinfo; pci.stageCount=2; pci.pStages=st; pci.pVertexInputState=&vi; pci.pInputAssemblyState=&ia; pci.pViewportState=&vp; pci.pRasterizationState=&rs; pci.pMultisampleState=&ms; pci.pDepthStencilState=&ds; pci.pColorBlendState=&cb; pci.pDynamicState=&dsi; pci.layout=pipe_tri_.layout; VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &pci, nullptr, &out.pipeline)); };
        make_pipeline(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, pipe_tri_);
        make_pipeline(VK_PRIMITIVE_TOPOLOGY_LINE_LIST, pipe_line_);
        make_pipeline(VK_PRIMITIVE_TOPOLOGY_POINT_LIST, pipe_point_);
        vkDestroyShaderModule(dev_, vs, nullptr); vkDestroyShaderModule(dev_, fs, nullptr);
    }

    void destroy_pipelines_(){ if (pipe_tri_.pipeline) vkDestroyPipeline(dev_, pipe_tri_.pipeline, nullptr); if (pipe_line_.pipeline) vkDestroyPipeline(dev_, pipe_line_.pipeline, nullptr); if (pipe_point_.pipeline) vkDestroyPipeline(dev_, pipe_point_.pipeline, nullptr); if (pipe_tri_.layout) vkDestroyPipelineLayout(dev_, pipe_tri_.layout, nullptr); pipe_tri_={}; pipe_line_={}; pipe_point_={}; }
    void destroy_gpu_buffers_(){ destroy_buffer_(pos_buf_); destroy_buffer_(tri_idx_); destroy_buffer_(line_idx_); }
};

int main(){ try{ VulkanEngine e; e.configure_window(1280, 720, "vx_xpbd_hina"); e.set_renderer(std::make_unique<HinaXPBDRenderer>()); e.init(); e.run(); e.cleanup(); } catch(const std::exception& ex){ std::fprintf(stderr, "Fatal: %s\n", ex.what()); return 1; } return 0; }

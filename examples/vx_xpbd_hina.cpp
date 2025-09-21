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
        build_gpu_buffers_(); build_shadow_resources_(); build_pipelines_();
        cam_.set_mode(vv::CameraMode::Orbit); auto s = cam_.state(); s.target={0,0,0}; s.distance=2.0f; s.pitch_deg=20.0f; s.yaw_deg=-120.0f; s.znear=0.01f; s.zfar=200.0f; cam_.set_state(s); frame_scene_to_positions_();
        sim_accum_ = 0.0;
    }

    void destroy(const EngineContext& e, const RendererCaps&) override {
        (void)e;
        destroy_gpu_buffers_(); destroy_pipelines_(); destroy_shadow_resources_(); if (solver_) sim::destroy(solver_); solver_ = nullptr; dev_ = VK_NULL_HANDLE; eng_ = {};
    }

    void update(const EngineContext&, const FrameContext& f) override {
        cam_.update(f.dt_sec, (int)f.extent.width, (int)f.extent.height); vp_w_=(int)f.extent.width; vp_h_=(int)f.extent.height;
        // simulation step
        if (params_.simulate && solver_) {
            sim_accum_ += f.dt_sec;
            double fixed = std::clamp<double>(params_.fixed_dt, 1.0/600.0, 1.0/30.0);
            int maxSteps = 4;
            while (sim_accum_ >= fixed && maxSteps--) { sim::step(solver_, (float)fixed); sim_accum_ -= fixed; }
        }
        // readback positions and upload
        if (!pos_buf_.mapped || !solver_) return;
        size_t outCount = 0; cpu_pos_.resize(3u*node_count_);
        sim::Status st = sim::copy_positions(solver_, cpu_pos_.data(), node_count_, &outCount);
        if (st == sim::Status::Ok && outCount >= node_count_) {
            std::memcpy(pos_buf_.mapped, cpu_pos_.data(), node_count_*sizeof(vv::float3));
            // compute and upload normals
            std::vector<float> cpu_normals; cpu_normals.reserve(cpu_pos_.size());
            compute_normals_(cpu_normals);
            if (!nrm_buf_.buf || nrm_buf_.size < node_count_*sizeof(vv::float3)) { destroy_buffer_(nrm_buf_); create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, nrm_buf_); }
            if (nrm_buf_.mapped && !cpu_normals.empty()) std::memcpy(nrm_buf_.mapped, cpu_normals.data(), node_count_*sizeof(vv::float3));
        }
        // Shadow map resize if changed
        if ((int)shadow_extent_.width != params_.shadow_size) { destroy_shadow_resources_(); build_shadow_resources_(); update_shadow_descriptor_(); }
    }

    void on_event(const SDL_Event& e, const EngineContext& eng, const FrameContext* f) override { cam_.handle_event(e, &eng, f); }

    void record_graphics(VkCommandBuffer cmd, const EngineContext&, const FrameContext& f) override {
        if (f.color_attachments.empty() || !pipe_tri_.pipeline) return;
        const auto& color = f.color_attachments.front(); const auto* depth = f.depth_attachment;
        auto barrier_img = [&](VkImage img, VkImageAspectFlags aspect, VkImageLayout oldL, VkImageLayout newL, VkPipelineStageFlags2 src, VkPipelineStageFlags2 dst, VkAccessFlags2 sa, VkAccessFlags2 da){
            VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2}; b.srcStageMask=src; b.dstStageMask=dst; b.srcAccessMask=sa; b.dstAccessMask=da; b.oldLayout=oldL; b.newLayout=newL; b.image=img; b.subresourceRange={aspect,0,1,0,1}; VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO}; di.imageMemoryBarrierCount=1; di.pImageMemoryBarriers=&b; vkCmdPipelineBarrier2(cmd,&di);
        };

        // 0) Update light matrices
        update_light_matrices_();
        const vv::float4x4 V = cam_.view_matrix(); const vv::float4x4 P = cam_.proj_matrix(); vv::float4x4 MVP = vv::mul(P, V);
        struct PCFull { float mvp[16]; float lightMVP[16]; float color[4]; float pointSize; float outline; float shadowStrength; float shadowTexel; } pc{};
        std::memcpy(pc.mvp, MVP.m.data(), sizeof(pc.mvp)); std::memcpy(pc.lightMVP, light_mvp_.m.data(), sizeof(pc.lightMVP)); pc.pointSize = params_.point_size; pc.outline = params_.outline_thickness; pc.shadowStrength = params_.shadow_strength; pc.shadowTexel = 1.0f / std::max(1, params_.shadow_size);

        // 1) Shadow map pass (depth-only)
        if (params_.enable_shadow && pipe_shadow_.pipeline && shadow_image_) {
            barrier_img(shadow_image_, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT, 0, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
            VkClearValue clear_depth{.depthStencil={1.0f,0}};
            VkRenderingAttachmentInfo da{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO}; da.imageView=shadow_view_; da.imageLayout=VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL; da.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; da.storeOp=VK_ATTACHMENT_STORE_OP_STORE; da.clearValue=clear_depth;
            VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea={{0,0}, {shadow_extent_.width, shadow_extent_.height}}; ri.layerCount=1; ri.colorAttachmentCount=0; ri.pColorAttachments=nullptr; ri.pDepthAttachment = &da; vkCmdBeginRendering(cmd,&ri);
            VkViewport vp{}; vp.x=0; vp.y=0; vp.width=(float)shadow_extent_.width; vp.height=(float)shadow_extent_.height; vp.minDepth=0; vp.maxDepth=1; VkRect2D sc{{0,0}, {shadow_extent_.width, shadow_extent_.height}}; vkCmdSetViewport(cmd,0,1,&vp); vkCmdSetScissor(cmd,0,1,&sc);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_shadow_.pipeline);
            vkCmdPushConstants(cmd, pipe_shadow_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PCFull), &pc);
            VkDeviceSize off0=0; vkCmdBindVertexBuffers(cmd, 0, 1, &pos_buf_.buf, &off0);
            vkCmdBindIndexBuffer(cmd, tri_idx_.buf, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, tri_count_, 1, 0, 0, 0);
            vkCmdEndRendering(cmd);
            barrier_img(shadow_image_, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }

        // 2) Main pass
        barrier_img(color.image, color.aspect, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        if (depth) barrier_img(depth->image, depth->aspect, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT, 0, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
        VkClearValue clear_color{.color={{0.05f,0.06f,0.07f,1.0f}}}; VkClearValue clear_depth{.depthStencil={1.0f,0}};
        VkRenderingAttachmentInfo ca{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO}; ca.imageView=color.view; ca.imageLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; ca.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; ca.storeOp=VK_ATTACHMENT_STORE_OP_STORE; ca.clearValue=clear_color;
        VkRenderingAttachmentInfo da{}; if (depth){ da.sType=VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO; da.imageView=depth->view; da.imageLayout=VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL; da.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; da.storeOp=VK_ATTACHMENT_STORE_OP_STORE; da.clearValue=clear_depth; }
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea={{0,0}, f.extent}; ri.layerCount=1; ri.colorAttachmentCount=1; ri.pColorAttachments=&ca; ri.pDepthAttachment = depth? &da : nullptr; vkCmdBeginRendering(cmd,&ri);
        VkViewport vp{}; vp.x=0; vp.y=0; vp.width=(float)f.extent.width; vp.height=(float)f.extent.height; vp.minDepth=0; vp.maxDepth=1; VkRect2D sc{{0,0}, f.extent}; vkCmdSetViewport(cmd,0,1,&vp); vkCmdSetScissor(cmd,0,1,&sc);
        // Mesh with lit+shadow
        if (params_.show_mesh){
            pc.color[0]=0.75f; pc.color[1]=0.82f; pc.color[2]=0.95f; pc.color[3]=1.0f;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_tri_.pipeline);
            if (params_.enable_shadow && shadow_ds_) vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_tri_.layout, 0, 1, &shadow_ds_, 0, nullptr);
            vkCmdPushConstants(cmd, pipe_tri_.layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PCFull), &pc);
            VkBuffer bufs[2] = { pos_buf_.buf, nrm_buf_.buf }; VkDeviceSize offs[2] = { 0, 0 };
            vkCmdBindVertexBuffers(cmd, 0, 2, bufs, offs); vkCmdBindIndexBuffer(cmd, tri_idx_.buf, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, tri_count_, 1, 0, 0, 0);
            // Optional outline
            if (params_.enable_outline && params_.outline_thickness>0.0f && pipe_outline_.pipeline){
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_outline_.pipeline);
                vkCmdPushConstants(cmd, pipe_outline_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PCFull), &pc);
                vkCmdBindVertexBuffers(cmd, 0, 2, bufs, offs); vkCmdBindIndexBuffer(cmd, tri_idx_.buf, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmd, tri_count_, 1, 0, 0, 0);
            }
        }
        // Constraints
        if (params_.show_constraints){
            struct PCLine { float mvp[16]; float color[4]; float pointSize; float _pad[3]; } pcl{}; std::memcpy(pcl.mvp, MVP.m.data(), sizeof(pcl.mvp));
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_line_.pipeline);
            // structural: light gray
            pcl.color[0]=0.85f; pcl.color[1]=0.85f; pcl.color[2]=0.85f; pcl.color[3]=1.0f; vkCmdPushConstants(cmd, pipe_line_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PCLine), &pcl);
            if (line_struct_count_>0){ VkDeviceSize off0 = 0; vkCmdBindVertexBuffers(cmd, 0, 1, &pos_buf_.buf, &off0); vkCmdBindIndexBuffer(cmd, line_struct_idx_.buf, 0, VK_INDEX_TYPE_UINT32); vkCmdDrawIndexed(cmd, line_struct_count_, 1, 0, 0, 0); }
            // shear: cyan
            pcl.color[0]=0.6f; pcl.color[1]=0.9f; pcl.color[2]=1.0f; pcl.color[3]=1.0f; vkCmdPushConstants(cmd, pipe_line_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PCLine), &pcl);
            if (line_shear_count_>0){ VkDeviceSize off1 = 0; vkCmdBindVertexBuffers(cmd, 0, 1, &pos_buf_.buf, &off1); vkCmdBindIndexBuffer(cmd, line_shear_idx_.buf, 0, VK_INDEX_TYPE_UINT32); vkCmdDrawIndexed(cmd, line_shear_count_, 1, 0, 0, 0); }
        }
        // Vertices
        if (params_.show_vertices){
            struct PCPoint { float mvp[16]; float color[4]; float pointSize; float _pad[3]; } pcp{}; std::memcpy(pcp.mvp, MVP.m.data(), sizeof(pcp.mvp)); pcp.color[0]=1.0f; pcp.color[1]=1.0f; pcp.color[2]=1.0f; pcp.color[3]=1.0f; pcp.pointSize = params_.point_size;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_point_.pipeline);
            vkCmdPushConstants(cmd, pipe_point_.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PCPoint), &pcp);
            VkDeviceSize offp = 0; vkCmdBindVertexBuffers(cmd, 0, 1, &pos_buf_.buf, &offp);
            vkCmdDraw(cmd, node_count_, 1, 0, 0);
        }
        vkCmdEndRendering(cmd);
        barrier_img(color.image, color.aspect, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT);
    }

    void on_imgui(const EngineContext& eng, const FrameContext&) override {
        auto* host = static_cast<vv_ui::TabsHost*>(eng.services);
        if (host) { host->add_overlay([this]{ cam_.imgui_draw_nav_overlay_space_tint(); }); host->add_overlay([this]{ cam_.imgui_draw_mini_axis_gizmo(); }); }
        if (!host) return;
        host->add_tab("HinaCloth XPBD", [this]{
            ImGui::Checkbox("Simulate", &params_.simulate); ImGui::SameLine(); if (ImGui::Button("Step")) { sim::step(solver_, std::clamp<float>((float)params_.fixed_dt, 1.f/600.f, 1.f/30.f)); }
            ImGui::SameLine(); if (ImGui::Button("Reset")) { reset_sim_(); frame_scene_to_positions_(); }
            ImGui::Separator();
            ImGui::Checkbox("Mesh", &params_.show_mesh); ImGui::SameLine(); ImGui::Checkbox("Vertices", &params_.show_vertices); ImGui::SameLine(); ImGui::Checkbox("Constraints", &params_.show_constraints);
            ImGui::SliderFloat("Point Size", &params_.point_size, 1.0f, 12.0f);
            ImGui::Separator();
            // Lighting & Shadow
            ImGui::Checkbox("Shadow", &params_.enable_shadow); ImGui::SameLine();
            const char* sizes[] = {"512","1024","2048"}; int cur = (params_.shadow_size==2048?2:params_.shadow_size==1024?1:0); if (ImGui::Combo("ShadowMap", &cur, sizes, IM_ARRAYSIZE(sizes))) { params_.shadow_size = (cur==2?2048:(cur==1?1024:512)); }
            ImGui::SliderFloat("Shadow Strength", &params_.shadow_strength, 0.0f, 1.0f);
            ImGui::Checkbox("Outline", &params_.enable_outline); ImGui::SameLine(); ImGui::SliderFloat("Outline Offset", &params_.outline_thickness, 0.0f, 0.02f, "%.4f");
            ImGui::Separator(); ImGui::SliderFloat("Fixed dt (s)", &params_.fixed_dt, 1.0f/240.0f, 1.0f/30.0f, "%.4f");
            ImGui::Separator(); ImGui::InputInt("Grid X", &params_.grid_x); ImGui::SameLine(); ImGui::InputInt("Grid Y", &params_.grid_y); ImGui::SliderFloat("Spacing", &params_.spacing, 0.02f, 0.2f);
            if (ImGui::Button("Rebuild Grid")) { rebuild_grid_(params_.grid_x, params_.grid_y, params_.spacing); frame_scene_to_positions_(); }
            ImGui::SameLine(); if (ImGui::Button("Frame Cloth")) { frame_scene_to_positions_(); }
        });
        host->add_tab("Camera", [this]{ cam_.imgui_panel_contents(); });
    }

private:
    struct Params { bool simulate{true}; float fixed_dt{1.0f/120.0f}; bool show_mesh{true}; bool show_vertices{true}; bool show_constraints{false}; float point_size{5.0f}; int grid_x{20}, grid_y{20}; float spacing{0.06f}; bool enable_shadow{true}; int shadow_size{1024}; float shadow_strength{0.85f}; bool enable_outline{false}; float outline_thickness{0.006f}; } params_{};

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
    GpuBuffer nrm_buf_{}; // vec3 normals
    GpuBuffer tri_idx_{}; uint32_t tri_count_{0};
    GpuBuffer line_struct_idx_{}; uint32_t line_struct_count_{0};
    GpuBuffer line_shear_idx_{};  uint32_t line_shear_count_{0};

    struct Pipeline { VkPipeline pipeline{}; VkPipelineLayout layout{}; };
    Pipeline pipe_tri_{}, pipe_line_{}, pipe_point_{}, pipe_outline_{}, pipe_shadow_{};

    // Shadow resources
    VkImage shadow_image_{VK_NULL_HANDLE}; VmaAllocation shadow_alloc_{nullptr}; VkImageView shadow_view_{VK_NULL_HANDLE}; VkExtent2D shadow_extent_{1024,1024}; VkSampler shadow_sampler_{VK_NULL_HANDLE};
    VkDescriptorSetLayout shadow_dsl_{VK_NULL_HANDLE}; VkDescriptorSet shadow_ds_{VK_NULL_HANDLE};
    vv::float4x4 light_mvp_{};

    double sim_accum_{0.0};

private:
    // Compute per-vertex normals (area-weighted)
    void compute_normals_(std::vector<float>& out_normals){
        out_normals.assign(3u*node_count_, 0.0f);
        auto accum = [&](uint32_t a, uint32_t b, uint32_t c){
            float ax = cpu_pos_[3u*a+0], ay = cpu_pos_[3u*a+1], az = cpu_pos_[3u*a+2];
            float bx = cpu_pos_[3u*b+0], by = cpu_pos_[3u*b+1], bz = cpu_pos_[3u*b+2];
            float cx = cpu_pos_[3u*c+0], cy = cpu_pos_[3u*c+1], cz = cpu_pos_[3u*c+2];
            float ux = bx-ax, uy = by-ay, uz = bz-az;
            float vx = cx-ax, vy = cy-ay, vz = cz-az;
            float nx = uy*vz - uz*vy; float ny = uz*vx - ux*vz; float nz = ux*vy - uy*vx;
            out_normals[3u*a+0]+=nx; out_normals[3u*a+1]+=ny; out_normals[3u*a+2]+=nz;
            out_normals[3u*b+0]+=nx; out_normals[3u*b+1]+=ny; out_normals[3u*b+2]+=nz;
            out_normals[3u*c+0]+=nx; out_normals[3u*c+1]+=ny; out_normals[3u*c+2]+=nz;
        };
        // walk triangles
        if (tri_idx_.mapped && tri_count_>=3){ const uint32_t* idx = static_cast<const uint32_t*>(tri_idx_.mapped); for (uint32_t i=0; i+2<tri_count_; i+=3){ accum(idx[i+0], idx[i+1], idx[i+2]); } }
        // normalize
        for(uint32_t i=0;i<node_count_;++i){ float nx = out_normals[3u*i+0], ny = out_normals[3u*i+1], nz = out_normals[3u*i+2]; float len = std::sqrt(nx*nx+ny*ny+nz*nz) + 1e-20f; out_normals[3u*i+0] = nx/len; out_normals[3u*i+1] = ny/len; out_normals[3u*i+2] = nz/len; }
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
        // line sets: structural (H/V) and shear (diagonals)
        std::vector<uint32_t> Ls; Ls.reserve(nx_*ny_*2);
        std::vector<uint32_t> Ld; Ld.reserve(nx_*ny_*2);
        for(uint32_t j=0;j<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ uint32_t a=id(i,j), b=id(i+1,j); Ls.push_back(a); Ls.push_back(b);} }
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i<nx_;++i){ uint32_t a=id(i,j), b=id(i,j+1); Ls.push_back(a); Ls.push_back(b);} }
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ uint32_t a=id(i,j), b=id(i+1,j+1); Ld.push_back(a); Ld.push_back(b); uint32_t c=id(i+1,j), d=id(i,j+1); Ld.push_back(c); Ld.push_back(d);} }
        line_struct_count_ = (uint32_t)Ls.size(); line_shear_count_ = (uint32_t)Ld.size();
        if (!line_struct_idx_.buf) create_buffer_(Ls.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_struct_idx_);
        else if (line_struct_idx_.size < Ls.size()*sizeof(uint32_t)) { destroy_buffer_(line_struct_idx_); create_buffer_(Ls.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_struct_idx_); }
        if (!Ls.empty()) std::memcpy(line_struct_idx_.mapped, Ls.data(), Ls.size()*sizeof(uint32_t));
        if (!line_shear_idx_.buf) create_buffer_(Ld.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_shear_idx_);
        else if (line_shear_idx_.size < Ld.size()*sizeof(uint32_t)) { destroy_buffer_(line_shear_idx_); create_buffer_(Ld.size()*sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, line_shear_idx_); }
        if (!Ld.empty()) std::memcpy(line_shear_idx_.mapped, Ld.data(), Ld.size()*sizeof(uint32_t));
    }

    void build_gpu_buffers_(){ if (!pos_buf_.buf) create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, pos_buf_); if (!nrm_buf_.buf) create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, nrm_buf_); rebuild_indices_(); }

    void build_pipelines_(){
        std::string dir(SHADER_OUTPUT_DIR);
        // Triangles: lit+shadow
        VkShaderModule vs_tri = make_shader(dev_, load_spv(dir+"/cloth_shadow.vert.spv"));
        VkShaderModule fs_tri = make_shader(dev_, load_spv(dir+"/cloth_shadow.frag.spv"));
        VkPipelineShaderStageCreateInfo st_tri[2]{}; for(int i=0;i<2;++i) st_tri[i].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; st_tri[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st_tri[0].module=vs_tri; st_tri[0].pName="main"; st_tri[1]=st_tri[0]; st_tri[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st_tri[1].module=fs_tri;
        VkVertexInputBindingDescription bind_pos{0, sizeof(vv::float3), VK_VERTEX_INPUT_RATE_VERTEX}; VkVertexInputBindingDescription bind_nrm{1, sizeof(vv::float3), VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputBindingDescription binds_tri[2] = {bind_pos, bind_nrm};
        VkVertexInputAttributeDescription attrs_tri[2] = { {0,0,VK_FORMAT_R32G32B32_SFLOAT,0}, {1,1,VK_FORMAT_R32G32B32_SFLOAT,0} };
        VkPipelineVertexInputStateCreateInfo vi_tri{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vi_tri.vertexBindingDescriptionCount=2; vi_tri.pVertexBindingDescriptions=binds_tri; vi_tri.vertexAttributeDescriptionCount=2; vi_tri.pVertexAttributeDescriptions=attrs_tri;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_BACK_BIT; rs.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth=1.0f;
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO}; ds.depthTestEnable=VK_TRUE; ds.depthWriteEnable=VK_TRUE; ds.depthCompareOp=VK_COMPARE_OP_LESS;
        VkPipelineColorBlendAttachmentState ba{}; ba.colorWriteMask=0xF; VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&ba;
        const VkDynamicState dyns[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR }; VkPipelineDynamicStateCreateInfo dsi{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; dsi.dynamicStateCount=2; dsi.pDynamicStates=dyns;
        // Descriptor set layout for shadow map
        VkDescriptorSetLayoutBinding s{.binding=0,.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,.descriptorCount=1,.stageFlags=VK_SHADER_STAGE_FRAGMENT_BIT}; VkDescriptorSetLayoutCreateInfo dslci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO}; dslci.bindingCount=1; dslci.pBindings=&s; VK_CHECK(vkCreateDescriptorSetLayout(dev_, &dslci, nullptr, &shadow_dsl_));
        // Push constants layout
        VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, 160}; VkPipelineLayoutCreateInfo lci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; lci.setLayoutCount=1; lci.pSetLayouts=&shadow_dsl_; lci.pushConstantRangeCount=1; lci.pPushConstantRanges=&pcr; VK_CHECK(vkCreatePipelineLayout(dev_, &lci, nullptr, &pipe_tri_.layout));
        VkPipelineRenderingCreateInfo rinfo{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; rinfo.colorAttachmentCount=1; rinfo.pColorAttachmentFormats=&color_fmt_; rinfo.depthAttachmentFormat=depth_fmt_;
        auto make_pipeline = [&](VkPrimitiveTopology topo, const VkPipelineShaderStageCreateInfo* st, uint32_t stageCount, const VkPipelineVertexInputStateCreateInfo* vi, const VkPipelineRasterizationStateCreateInfo& rsIn, const VkPipelineDepthStencilStateCreateInfo& dsIn, Pipeline& out){ VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=topo; VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO}; pci.pNext=&rinfo; pci.stageCount=stageCount; pci.pStages=st; pci.pVertexInputState=vi; pci.pInputAssemblyState=&ia; pci.pViewportState=&vp; pci.pRasterizationState=&rsIn; pci.pMultisampleState=&ms; pci.pDepthStencilState=&dsIn; pci.pColorBlendState=&cb; pci.pDynamicState=&dsi; pci.layout=pipe_tri_.layout; VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &pci, nullptr, &out.pipeline)); };
        make_pipeline(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, st_tri, 2, &vi_tri, rs, ds, pipe_tri_);
        vkDestroyShaderModule(dev_, vs_tri, nullptr); vkDestroyShaderModule(dev_, fs_tri, nullptr);

        // Lines/points: basic shaders (existing)
        VkShaderModule vs_basic = make_shader(dev_, load_spv(dir+"/cloth.vert.spv")); VkShaderModule fs_basic = make_shader(dev_, load_spv(dir+"/cloth.frag.spv"));
        VkPipelineShaderStageCreateInfo st_basic[2]{}; for(int i=0;i<2;++i) st_basic[i].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; st_basic[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st_basic[0].module=vs_basic; st_basic[0].pName="main"; st_basic[1]=st_basic[0]; st_basic[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st_basic[1].module=fs_basic;
        VkPipelineVertexInputStateCreateInfo vi_basic{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vi_basic.vertexBindingDescriptionCount=1; vi_basic.pVertexBindingDescriptions=&bind_pos; VkVertexInputAttributeDescription attr_pos{0,0,VK_FORMAT_R32G32B32_SFLOAT,0}; vi_basic.vertexAttributeDescriptionCount=1; vi_basic.pVertexAttributeDescriptions=&attr_pos;
        pipe_line_.layout = pipe_tri_.layout; pipe_point_.layout = pipe_tri_.layout; pipe_outline_.layout = pipe_tri_.layout; // share
        auto make_pipeline_basic = [&](VkPrimitiveTopology topo, Pipeline& out){ VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=topo; VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO}; pci.pNext=&rinfo; pci.stageCount=2; pci.pStages=st_basic; pci.pVertexInputState=&vi_basic; pci.pInputAssemblyState=&ia; pci.pViewportState=&vp; pci.pRasterizationState=&rs; pci.pMultisampleState=&ms; pci.pDepthStencilState=&ds; pci.pColorBlendState=&cb; pci.pDynamicState=&dsi; pci.layout=pipe_line_.layout; VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &pci, nullptr, &out.pipeline)); };
        make_pipeline_basic(VK_PRIMITIVE_TOPOLOGY_LINE_LIST, pipe_line_);
        make_pipeline_basic(VK_PRIMITIVE_TOPOLOGY_POINT_LIST, pipe_point_);
        vkDestroyShaderModule(dev_, vs_basic, nullptr); vkDestroyShaderModule(dev_, fs_basic, nullptr);

        // Outline pipeline: backface, no depth write
        VkShaderModule vs_ol = make_shader(dev_, load_spv(dir+"/cloth_outline.vert.spv")); VkShaderModule fs_ol = make_shader(dev_, load_spv(dir+"/cloth_outline.frag.spv"));
        VkPipelineShaderStageCreateInfo st_ol[2]{}; for(int i=0;i<2;++i) st_ol[i].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; st_ol[0].stage=VK_SHADER_STAGE_VERTEX_BIT; st_ol[0].module=vs_ol; st_ol[0].pName="main"; st_ol[1]=st_ol[0]; st_ol[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; st_ol[1].module=fs_ol;
        VkPipelineRasterizationStateCreateInfo rs_ol = rs; rs_ol.cullMode = VK_CULL_MODE_FRONT_BIT; VkPipelineDepthStencilStateCreateInfo ds_ol = ds; ds_ol.depthWriteEnable = VK_FALSE;
        make_pipeline(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, st_ol, 2, &vi_tri, rs_ol, ds_ol, pipe_outline_);
        vkDestroyShaderModule(dev_, vs_ol, nullptr); vkDestroyShaderModule(dev_, fs_ol, nullptr);

        // Shadow depth-only pipeline (VS only)
        VkShaderModule vs_sd = make_shader(dev_, load_spv(dir+"/shadow_depth.vert.spv")); VkPipelineShaderStageCreateInfo st_sd{}; st_sd.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; st_sd.stage=VK_SHADER_STAGE_VERTEX_BIT; st_sd.module=vs_sd; st_sd.pName="main";
        VkPipelineRenderingCreateInfo rinfo_sd{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; rinfo_sd.colorAttachmentCount=0; rinfo_sd.depthAttachmentFormat=depth_fmt_;
        VkPipelineInputAssemblyStateCreateInfo ia_sd{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia_sd.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkGraphicsPipelineCreateInfo pci_sd{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO}; pci_sd.pNext=&rinfo_sd; pci_sd.stageCount=1; pci_sd.pStages=&st_sd; pci_sd.pVertexInputState=&vi_basic; pci_sd.pInputAssemblyState=&ia_sd; pci_sd.pViewportState=&vp; pci_sd.pRasterizationState=&rs; pci_sd.pMultisampleState=&ms; pci_sd.pDepthStencilState=&ds; pci_sd.pColorBlendState=nullptr; pci_sd.pDynamicState=&dsi; pci_sd.layout=pipe_tri_.layout; VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &pci_sd, nullptr, &pipe_shadow_.pipeline)); pipe_shadow_.layout = pipe_tri_.layout; vkDestroyShaderModule(dev_, vs_sd, nullptr);

        update_shadow_descriptor_();
    }

    void destroy_pipelines_(){ if (pipe_tri_.pipeline) vkDestroyPipeline(dev_, pipe_tri_.pipeline, nullptr); if (pipe_line_.pipeline) vkDestroyPipeline(dev_, pipe_line_.pipeline, nullptr); if (pipe_point_.pipeline) vkDestroyPipeline(dev_, pipe_point_.pipeline, nullptr); if (pipe_outline_.pipeline) vkDestroyPipeline(dev_, pipe_outline_.pipeline, nullptr); if (pipe_shadow_.pipeline) vkDestroyPipeline(dev_, pipe_shadow_.pipeline, nullptr); if (pipe_tri_.layout) vkDestroyPipelineLayout(dev_, pipe_tri_.layout, nullptr); pipe_tri_={}; pipe_line_={}; pipe_point_={}; pipe_outline_={}; pipe_shadow_={}; if (shadow_dsl_) { vkDestroyDescriptorSetLayout(dev_, shadow_dsl_, nullptr); shadow_dsl_ = VK_NULL_HANDLE; } if (shadow_sampler_) { vkDestroySampler(dev_, shadow_sampler_, nullptr); shadow_sampler_ = VK_NULL_HANDLE; } }
    void destroy_gpu_buffers_(){ destroy_buffer_(pos_buf_); destroy_buffer_(nrm_buf_); destroy_buffer_(tri_idx_); destroy_buffer_(line_struct_idx_); destroy_buffer_(line_shear_idx_); }

    void build_shadow_resources_(){ shadow_extent_ = { (uint32_t)params_.shadow_size, (uint32_t)params_.shadow_size }; VkImageCreateInfo imgci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO}; imgci.imageType=VK_IMAGE_TYPE_2D; imgci.format=depth_fmt_; imgci.extent={shadow_extent_.width, shadow_extent_.height, 1}; imgci.mipLevels=1; imgci.arrayLayers=1; imgci.samples=VK_SAMPLE_COUNT_1_BIT; imgci.tiling=VK_IMAGE_TILING_OPTIMAL; imgci.usage=VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; imgci.sharingMode=VK_SHARING_MODE_EXCLUSIVE; imgci.initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; VmaAllocationCreateInfo ai{}; ai.usage = VMA_MEMORY_USAGE_AUTO; VK_CHECK(vmaCreateImage(eng_.allocator, &imgci, &ai, &shadow_image_, &shadow_alloc_, nullptr)); VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO}; vci.image=shadow_image_; vci.viewType=VK_IMAGE_VIEW_TYPE_2D; vci.format=depth_fmt_; vci.subresourceRange={VK_IMAGE_ASPECT_DEPTH_BIT,0,1,0,1}; VK_CHECK(vkCreateImageView(dev_, &vci, nullptr, &shadow_view_)); VkSamplerCreateInfo sci{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO}; sci.magFilter=VK_FILTER_LINEAR; sci.minFilter=VK_FILTER_LINEAR; sci.mipmapMode=VK_SAMPLER_MIPMAP_MODE_NEAREST; sci.addressModeU=sci.addressModeV=sci.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; sci.compareEnable=VK_FALSE; sci.borderColor=VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE; VK_CHECK(vkCreateSampler(dev_, &sci, nullptr, &shadow_sampler_)); }

    void destroy_shadow_resources_(){ if (shadow_view_) { vkDestroyImageView(dev_, shadow_view_, nullptr); shadow_view_ = VK_NULL_HANDLE; } if (shadow_image_) { vmaDestroyImage(eng_.allocator, shadow_image_, shadow_alloc_); shadow_image_ = VK_NULL_HANDLE; shadow_alloc_ = nullptr; } }

    void update_shadow_descriptor_(){ if (!shadow_dsl_ || !eng_.descriptorAllocator) return; if (!shadow_ds_) { VkDescriptorSetLayout layouts[] = {shadow_dsl_}; VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO}; ai.descriptorPool = eng_.descriptorAllocator->pool; ai.descriptorSetCount=1; ai.pSetLayouts=layouts; VK_CHECK(vkAllocateDescriptorSets(dev_, &ai, &shadow_ds_)); } VkDescriptorImageInfo ii{.sampler=shadow_sampler_, .imageView=shadow_view_, .imageLayout=VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; VkWriteDescriptorSet wr{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET}; wr.dstSet=shadow_ds_; wr.dstBinding=0; wr.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; wr.descriptorCount=1; wr.pImageInfo=&ii; vkUpdateDescriptorSets(dev_, 1, &wr, 0, nullptr); }

    void build_shadow_camera_(vv::float4x4& out_view, vv::float4x4& out_proj){ vv::float3 light_dir = vv::normalize(vv::make_float3(0.4f, 0.7f, 0.5f)); vv::float3 center{0,0,0}; vv::float3 eye{ -light_dir.x*3.0f, -light_dir.y*3.0f, -light_dir.z*3.0f }; out_view = vv::make_look_at(eye, center, vv::make_float3(0,1,0)); float r=1.5f; out_proj = vv::make_ortho(-r,r,-r,r, 0.01f, 10.0f); }
    void update_light_matrices_(){ vv::float4x4 lv, lp; build_shadow_camera_(lv, lp); light_mvp_ = vv::mul(lp, lv); }

    // GPU buffer helpers
    void create_buffer_(VkDeviceSize sz, VkBufferUsageFlags usage, VmaMemoryUsage memUsage, bool mapped, GpuBuffer& out){ VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bi.size=sz; bi.usage=usage; bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE; VmaAllocationCreateInfo ai{}; ai.usage = memUsage; ai.flags = mapped? VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT : 0; VK_CHECK(vmaCreateBuffer(eng_.allocator, &bi, &ai, &out.buf, &out.alloc, nullptr)); out.size=(size_t)sz; out.mapped=nullptr; if (mapped) { vmaMapMemory(eng_.allocator, out.alloc, &out.mapped); } }
    void destroy_buffer_(GpuBuffer& b){ if (b.mapped) { vmaUnmapMemory(eng_.allocator, b.alloc); b.mapped=nullptr; } if (b.buf) vmaDestroyBuffer(eng_.allocator, b.buf, b.alloc); b = {}; }

    void build_sim_(){ rebuild_grid_(params_.grid_x, params_.grid_y, params_.spacing); }
    void reset_sim_(){ rebuild_grid_(nx_, ny_, dx_); }

    void rebuild_grid_(uint32_t nx, uint32_t ny, float dx){
        if (solver_) { sim::destroy(solver_); solver_ = nullptr; }
        destroy_buffer_(tri_idx_); destroy_buffer_(line_struct_idx_); destroy_buffer_(line_shear_idx_);
        nx_ = std::max(2u, nx); ny_ = std::max(2u, ny); dx_ = dx; node_count_ = nx_*ny_;
        std::vector<float> pos(3u*node_count_), vel(3u*node_count_, 0.0f);
        for(uint32_t j=0;j<ny_;++j){ for(uint32_t i=0;i<nx_;++i){ uint32_t id=vid(i,j,nx_); pos[3u*id+0] = (float)i*dx_; pos[3u*id+1] = 0.8f; pos[3u*id+2] = (float)j*dx_; }}
        edges_.clear(); edges_.reserve(nx_*ny_*4);
        auto idf = [&](uint32_t i,uint32_t j){ return vid(i,j,nx_); };
        for(uint32_t j=0;j<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ edges_.push_back(idf(i,j)); edges_.push_back(idf(i+1,j)); }}
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i<nx_;++i){ edges_.push_back(idf(i,j)); edges_.push_back(idf(i,j+1)); }}
        for(uint32_t j=0;j+1<ny_;++j){ for(uint32_t i=0;i+1<nx_;++i){ edges_.push_back(idf(i,j)); edges_.push_back(idf(i+1,j+1)); edges_.push_back(idf(i+1,j)); edges_.push_back(idf(i,j+1)); }}
        sim::FieldView fpos{"position", sim::FieldType::F32, pos.data(), node_count_, 3, sizeof(float)*3}; sim::FieldView fvel{"velocity", sim::FieldType::F32, vel.data(), node_count_, 3, sizeof(float)*3}; sim::FieldView flds[2] = {fpos, fvel}; sim::StateInit st{flds, 2};
        sim::RelationView rel{edges_.data(), 2, edges_.size()/2, "edges"}; sim::TopologyIn topo{node_count_, &rel, 1}; const char* tags[] = {"edges"}; sim::FieldUse uses[] = {{"position", true}}; sim::OperatorDecl op{ "distance", tags, 1, uses, 1, sim::OpStage::Solve, true}; sim::OperatorsDecl ops{ &op, 1}; sim::Param gpy{"gravity_y", sim::ParamType::F32, {.f32=-9.8f}}; sim::Parameters params{&gpy, 1}; sim::Policy pol{{sim::DataLayout::Auto, sim::Backend::Auto, -1, true, true},{1, 10, 0.0f, sim::TimeStepper::Symplectic}}; sim::SpaceDesc sp{sim::SpaceType::Lagrangian, 1, 0}; sim::EventsScript ev{nullptr, 0}; sim::BuildDesc bd{st, params, topo, pol, sp, ops, ev, sim::ValidateLevel::Strict, {true, 8}}; auto r = sim::create(bd); if (r.status != sim::Status::Ok) throw std::runtime_error("HinaCloth create() failed"); solver_ = r.value;
        // Pin two top vertices (inv_mass=0)
        struct RegionPayload { const char* name; uint32_t start; uint32_t count; float v[3]; };
        uint32_t id0 = vid(0, 0, nx_); uint32_t id1 = vid(nx_-1, 0, nx_);
        RegionPayload pinA{"inv_mass", id0, 1u, {0.0f, 0.0f, 0.0f}}; RegionPayload pinB{"inv_mass", id1, 1u, {0.0f, 0.0f, 0.0f}};
        sim::Command cA{sim::CommandTag::SetFieldRegion, &pinA, sizeof(pinA)}; sim::Command cB{sim::CommandTag::SetFieldRegion, &pinB, sizeof(pinB)}; sim::push_command(solver_, cA); sim::push_command(solver_, cB); sim::flush_commands(solver_, sim::ApplyPhase::BeforeFrame);
        rebuild_indices_();
        if (!pos_buf_.buf || pos_buf_.size < node_count_*sizeof(vv::float3)) { destroy_buffer_(pos_buf_); create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, pos_buf_); }
        if (!nrm_buf_.buf || nrm_buf_.size < node_count_*sizeof(vv::float3)) { destroy_buffer_(nrm_buf_); create_buffer_(node_count_*sizeof(vv::float3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO, true, nrm_buf_); }
        cpu_pos_.assign(pos.begin(), pos.end()); if (pos_buf_.mapped) std::memcpy(pos_buf_.mapped, cpu_pos_.data(), node_count_*sizeof(vv::float3)); std::vector<float> cpu_normals; cpu_normals.reserve(cpu_pos_.size()); compute_normals_(cpu_normals); if (nrm_buf_.mapped && !cpu_normals.empty()) std::memcpy(nrm_buf_.mapped, cpu_normals.data(), node_count_*sizeof(vv::float3));
    }

    void frame_scene_to_positions_(){ if (node_count_==0 || cpu_pos_.empty()) return; vv::float3 mn{cpu_pos_[0], cpu_pos_[1], cpu_pos_[2]}, mx=mn; for(uint32_t i=0;i<node_count_;++i){ float x=cpu_pos_[3*i+0], y=cpu_pos_[3*i+1], z=cpu_pos_[3*i+2]; mn.x=std::min(mn.x,x); mn.y=std::min(mn.y,y); mn.z=std::min(mn.z,z); mx.x=std::max(mx.x,x); mx.y=std::max(mx.y,y); mx.z=std::max(mx.z,z);} mn.z-=0.2f; mx.z+=0.2f; cam_.set_scene_bounds(vv::BoundingBox{.min=mn,.max=mx,.valid=true}); cam_.frame_scene(1.12f); }

};

int main(){ try{ VulkanEngine e; e.configure_window(1280, 720, "vx_xpbd_hina"); e.set_renderer(std::make_unique<HinaXPBDRenderer>()); e.init(); e.run(); e.cleanup(); } catch(const std::exception& ex){ std::fprintf(stderr, "Fatal: %s\n", ex.what()); return 1; } return 0; }

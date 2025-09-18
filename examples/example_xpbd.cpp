#include "vk_engine.h"
#include <vulkan/vulkan.h>

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <algorithm>
#include <imgui.h>

#ifndef VK_CHECK
#define VK_CHECK(x) do { VkResult _res = (x); if (_res != VK_SUCCESS) { throw std::runtime_error(std::string("Vulkan error ")+ std::to_string(_res) + " at " #x); } } while(false)
#endif

// Where CMake places compiled SPIR-V
#ifndef SHADER_OUTPUT_DIR
#define SHADER_OUTPUT_DIR "./examples/shader"
#endif

// ----------------------------------------------------------------------------
// File utilities
// ----------------------------------------------------------------------------
static std::vector<char> load_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("Failed to open file: " + path);
    size_t sz = (size_t)f.tellg();
    f.seekg(0);
    std::vector<char> data(sz);
    f.read(data.data(), sz);
    return data;
}

static VkShaderModule create_shader_module(VkDevice device, const std::vector<char>& code) {
    VkShaderModuleCreateInfo ci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    ci.codeSize = code.size();
    ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule mod{};
    VK_CHECK(vkCreateShaderModule(device, &ci, nullptr, &mod));
    return mod;
}

// ----------------------------------------------------------------------------
// Simple VBO via VMA (host-visible, sequential-write per frame)
// ----------------------------------------------------------------------------
struct GpuBuffer {
    VkBuffer buffer{VK_NULL_HANDLE};
    VmaAllocation alloc{VK_NULL_HANDLE};
    size_t sizeBytes{0};
};

static void create_buffer(VmaAllocator allocator, VkDeviceSize size, VkBufferUsageFlags usage, GpuBuffer& out) {
    VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bci.size = size;
    bci.usage = usage;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo aci{};
    aci.usage = VMA_MEMORY_USAGE_AUTO;
    aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VmaAllocationInfo ainfo{};
    VK_CHECK(vmaCreateBuffer(allocator, &bci, &aci, &out.buffer, &out.alloc, &ainfo));
    out.sizeBytes = size;
}

static void destroy_buffer(VmaAllocator allocator, GpuBuffer& buf) {
    if (buf.buffer) {
        vmaDestroyBuffer(allocator, buf.buffer, buf.alloc);
        buf.buffer = VK_NULL_HANDLE;
        buf.alloc = VK_NULL_HANDLE;
        buf.sizeBytes = 0;
    }
}

// ----------------------------------------------------------------------------
// XPBD Cloth layouts & solvers
// ----------------------------------------------------------------------------
#include "xpbd_params.h"
#include "aos/cloth_data_aos.h"
#include "soa/cloth_data_soa.h"
#include "aosoa/cloth_data_aosoa.h"
#include "aligned/cloth_data_aligned.h"
#include "aos/solver_xpbd_aos.h"
#include "soa/solver_xpbd_soa.h"
#include "aosoa/solver_xpbd_aosoa.h"
#include "aligned/solver_xpbd_aligned.h"
#include "cloth_grid_utils.h"

struct Vertex {
    float x, y;      // NDC-space
    float r, g, b, a;
};

class XPBDRenderer final : public IRenderer {
public:
    void initialize(const EngineContext& eng) override {
        device_ = eng.device;
        allocator_ = eng.allocator;

        buildPipelines();
        initCloth();
        allocateBuffers();
    }

    void destroy(const EngineContext& eng) override {
        destroy_buffer(allocator_, vboLines_);
        destroy_buffer(allocator_, vboTris_);
        if (pipeLines_)   { vkDestroyPipeline(eng.device, pipeLines_, nullptr); pipeLines_ = VK_NULL_HANDLE; }
        if (pipeTris_)    { vkDestroyPipeline(eng.device, pipeTris_, nullptr); pipeTris_  = VK_NULL_HANDLE; }
        if (pipeLayout_)  { vkDestroyPipelineLayout(eng.device, pipeLayout_, nullptr); pipeLayout_ = VK_NULL_HANDLE; }
        device_ = VK_NULL_HANDLE;
        allocator_ = VK_NULL_HANDLE;
    }

    void get_capabilities(RendererCaps& caps) const override {
        caps = RendererCaps{};
        caps.uses_depth = VK_FALSE;
        caps.uses_offscreen = VK_TRUE;
        caps.dynamic_rendering = VK_TRUE;
    }

    void on_swapchain_ready(const EngineContext&, const FrameContext& frm) override {
        // fit-to-view scaling using swapchain aspect
        viewportW_ = (float)frm.extent.width;
        viewportH_ = (float)frm.extent.height;
    }

    void on_event(const SDL_Event& e, const EngineContext&, const FrameContext*) override {
        if (e.type == SDL_EVENT_KEY_DOWN) {
            SDL_Scancode sc = e.key.scancode;
            if (sc == SDL_SCANCODE_1) mode_ = 0; // AoS
            else if (sc == SDL_SCANCODE_2) mode_ = 1; // SoA
            else if (sc == SDL_SCANCODE_3) mode_ = 2; // AoSoA
            else if (sc == SDL_SCANCODE_4) mode_ = 3; // Aligned SoA
            else if (sc == SDL_SCANCODE_Q) backend_ = 0; // Native
            else if (sc == SDL_SCANCODE_W) backend_ = 1; // TBB
            else if (sc == SDL_SCANCODE_E) backend_ = 2; // AVX2
        }
    }

    void on_imgui(const EngineContext&, const FrameContext&) override {
        ImGui::Begin("XPBD Controls");
        const char* layouts[] = {"AoS","SoA","AoSoA","Aligned"};
        const char* backends[] = {"Native","TBB","AVX2"};
        ImGui::Combo("Layout (1/2/3/4)", &mode_, layouts, IM_ARRAYSIZE(layouts));
        ImGui::Combo("Backend (Q/W/E)", &backend_, backends, IM_ARRAYSIZE(backends));
        if (ImGui::Button("Reset")) resetCloth();
        ImGui::SameLine(); ImGui::Checkbox("Simulate", &simulate_);
        ImGui::SameLine(); if (ImGui::Button("Step")) { bool was = simulate_; simulate_ = true; on_step_once_ = true; (void)was; }
        ImGui::SliderFloat("Speed", &sim_speed_, 0.f, 4.f, "%.2fx");
        ImGui::SeparatorText("Params");
        ImGui::SliderInt("Iterations", &ui_params_.iterations, 1, 80);
        ImGui::SliderInt("Substeps", &ui_params_.substeps, 1, 8);
        ImGui::SliderFloat("dt min", &ui_params_.min_dt, 1e-4f, 5e-3f, "%.5f");
        ImGui::SliderFloat("dt max", &ui_params_.max_dt, 5e-3f, 5e-2f, "%.4f");
        ImGui::SliderFloat3("Gravity", &ui_params_.ax, -50.f, 50.f, "%.2f");
        ImGui::SliderFloat("Vel Damping", &ui_params_.velocity_damping, 0.f, 0.2f, "%.3f");
        ImGui::Checkbox("Warmstart", (bool*)&ui_params_.warmstart);
        ImGui::SliderFloat("Lambda Decay", &ui_params_.lambda_decay, 0.f, 1.f, "%.3f");
        ImGui::SeparatorText("Compliance Scale");
        ImGui::SliderFloat("All", &ui_params_.compliance_scale_all, 0.f, 10.f, "%.3f");
        ImGui::SliderFloat("Structural", &ui_params_.compliance_scale_structural, 0.f, 10.f, "%.3f");
        ImGui::SliderFloat("Shear", &ui_params_.compliance_scale_shear, 0.f, 10.f, "%.3f");
        ImGui::SliderFloat("Bending", &ui_params_.compliance_scale_bending, 0.f, 10.f, "%.3f");
        ImGui::SliderFloat("Max Correction", &ui_params_.max_correction, 0.f, 0.05f, "%.4f");
        ImGui::SeparatorText("Residuals");
        auto r = computeResidual();
        ImGui::Text("L1=%.4g  L2=%.4g  Linf=%.4g", r.l1, r.l2, r.linf);
        ImGui::Text("pts=%zu  cons=%zu", r.npts, r.ncons);
        ImGui::End();

        if (on_step_once_) { simulate_ = false; on_step_once_ = false; }
    }

    void update(const EngineContext&, const FrameContext& frm) override {
        const float dt = (float)frm.dt_sec * sim_speed_;
        if (simulate_) {
            const auto& params = ui_params_;
            auto use_native = backend_ == 0;
            auto use_tbb    = backend_ == 1;
            auto use_avx2   = backend_ == 2;
            if (mode_ == 0) {
                if (use_tbb) HinaPE::xpbd_step_tbb_aos(cloth_aos_, dt, params);
                else if (use_avx2) HinaPE::xpbd_step_avx2_aos(cloth_aos_, dt, params);
                else HinaPE::xpbd_step_native_aos(cloth_aos_, dt, params);
            } else if (mode_ == 1) {
                if (use_tbb) HinaPE::xpbd_step_tbb_soa(cloth_soa_, dt, params);
                else if (use_avx2) HinaPE::xpbd_step_avx2_soa(cloth_soa_, dt, params);
                else HinaPE::xpbd_step_native_soa(cloth_soa_, dt, params);
            } else if (mode_ == 2) {
                if (use_tbb) HinaPE::xpbd_step_tbb_aosoa(cloth_aosoa_, dt, params);
                else if (use_avx2) HinaPE::xpbd_step_avx2_aosoa(cloth_aosoa_, dt, params);
                else HinaPE::xpbd_step_native_aosoa(cloth_aosoa_, dt, params);
            } else {
                if (use_tbb) HinaPE::xpbd_step_tbb_aligned(cloth_aligned_, dt, params);
                else if (use_avx2) HinaPE::xpbd_step_avx2_aligned(cloth_aligned_, dt, params);
                else HinaPE::xpbd_step_native_aligned(cloth_aligned_, dt, params);
            }
        }
        buildGeometry();
        uploadGeometry();
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext&, const FrameContext& frm) override {
        // Transition to color attachment
        VkImageMemoryBarrier2 toColor{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toColor.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        toColor.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        toColor.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toColor.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toColor.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        toColor.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toColor.image = frm.offscreen_image;
        toColor.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep.imageMemoryBarrierCount = 1; dep.pImageMemoryBarriers = &toColor;
        vkCmdPipelineBarrier2(cmd, &dep);

        VkClearValue clear{ {{0.06f, 0.07f, 0.10f, 1.0f}} };
        VkRenderingAttachmentInfo att{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        att.imageView = frm.offscreen_image_view;
        att.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.clearValue = clear;

        VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        ri.renderArea = { {0,0}, frm.extent };
        ri.layerCount = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments = &att;
        vkCmdBeginRendering(cmd, &ri);

        // Flip Vulkan's default Y-down to Y-up so gravity appears downward
        VkViewport vp{}; vp.x = 0; vp.y = (float)frm.extent.height; vp.width = (float)frm.extent.width; vp.height = -(float)frm.extent.height; vp.minDepth=0.f; vp.maxDepth=1.f;
        VkRect2D sc{ {0,0}, frm.extent };
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);

        // Draw constraints (lines)
        if (lineVertCount_ > 0) {
            VkDeviceSize offs = 0;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLines_);
            vkCmdBindVertexBuffers(cmd, 0, 1, &vboLines_.buffer, &offs);
            vkCmdDraw(cmd, lineVertCount_, 1, 0, 0);
        }

        // Draw vertices (small quads as triangles)
        if (triVertCount_ > 0) {
            VkDeviceSize offs = 0;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeTris_);
            vkCmdBindVertexBuffers(cmd, 0, 1, &vboTris_.buffer, &offs);
            vkCmdDraw(cmd, triVertCount_, 1, 0, 0);
        }

        vkCmdEndRendering(cmd);

        // Back to GENERAL for engine blit
        VkImageMemoryBarrier2 toGeneral{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toGeneral.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toGeneral.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toGeneral.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        toGeneral.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
        toGeneral.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toGeneral.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        toGeneral.image = frm.offscreen_image;
        toGeneral.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        VkDependencyInfo dep2{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep2.imageMemoryBarrierCount = 1; dep2.pImageMemoryBarriers = &toGeneral;
        vkCmdPipelineBarrier2(cmd, &dep2);
    }

private:
    void buildPipelines() {
        // Common pipeline layout
        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        VK_CHECK(vkCreatePipelineLayout(device_, &plci, nullptr, &pipeLayout_));

        // Shader modules
        std::string base = std::string(SHADER_OUTPUT_DIR);
        auto vertBytes = load_file(base + "/xpbd.vert.spv");
        auto fragBytes = load_file(base + "/xpbd.frag.spv");
        VkShaderModule vmod = create_shader_module(device_, vertBytes);
        VkShaderModule fmod = create_shader_module(device_, fragBytes);

        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vmod;
        stages[0].pName  = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fmod;
        stages[1].pName  = "main";

        // Vertex layout
        VkVertexInputBindingDescription bind{}; bind.binding = 0; bind.stride = sizeof(Vertex); bind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        VkVertexInputAttributeDescription attrs[2]{};
        attrs[0].location = 0; attrs[0].binding = 0; attrs[0].format = VK_FORMAT_R32G32_SFLOAT;        attrs[0].offset = offsetof(Vertex, x);
        attrs[1].location = 1; attrs[1].binding = 0; attrs[1].format = VK_FORMAT_R32G32B32A32_SFLOAT; attrs[1].offset = offsetof(Vertex, r);
        VkPipelineVertexInputStateCreateInfo vi{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        vi.vertexBindingDescriptionCount = 1; vi.pVertexBindingDescriptions = &bind;
        vi.vertexAttributeDescriptionCount = 2; vi.pVertexAttributeDescriptions = attrs;

        VkPipelineInputAssemblyStateCreateInfo iaLines{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        iaLines.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
        VkPipelineInputAssemblyStateCreateInfo iaTris{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        iaTris.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo vp{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vp.viewportCount = 1; vp.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rs{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rs.polygonMode = VK_POLYGON_MODE_FILL; rs.cullMode = VK_CULL_MODE_NONE; rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth = 1.5f;

        VkPipelineMultisampleStateCreateInfo ms{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO }; ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState cbatt{}; cbatt.colorWriteMask = 0xF;
        VkPipelineColorBlendStateCreateInfo cb{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO }; cb.attachmentCount = 1; cb.pAttachments = &cbatt;

        VkDynamicState dynArr[2]{ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dyn{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dyn.dynamicStateCount = 2; dyn.pDynamicStates = dynArr;

        VkFormat colorFormat = VK_FORMAT_B8G8R8A8_UNORM;
        VkPipelineRenderingCreateInfo rendering{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        rendering.colorAttachmentCount = 1; rendering.pColorAttachmentFormats = &colorFormat;

        // Create 2 pipelines
        VkGraphicsPipelineCreateInfo gp{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        gp.pNext = &rendering;
        gp.stageCount = 2; gp.pStages = stages;
        gp.pVertexInputState = &vi;
        gp.pViewportState = &vp;
        gp.pRasterizationState = &rs;
        gp.pMultisampleState = &ms;
        gp.pColorBlendState = &cb;
        gp.pDynamicState = &dyn;
        gp.layout = pipeLayout_;
        gp.renderPass = VK_NULL_HANDLE;

        gp.pInputAssemblyState = &iaLines;
        VK_CHECK(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr, &pipeLines_));

        gp.pInputAssemblyState = &iaTris;
        VK_CHECK(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr, &pipeTris_));

        vkDestroyShaderModule(device_, vmod, nullptr);
        vkDestroyShaderModule(device_, fmod, nullptr);
    }

    void initCloth() {
        clothNx_ = 40;
        clothNy_ = 25;
        rebuildClothState();
    }

    void resetCloth() {
        rebuildClothState();
        buildGeometry();
        uploadGeometry();
    }

    void rebuildClothState() {
        const float clothWidth = 1.6f;
        const float clothHeight = 1.0f;
        const float clothStartY = 0.3f;
        const bool pinTopCorners = true;
        HinaPE::build_cloth_grid_aos(cloth_aos_, clothNx_, clothNy_, clothWidth, clothHeight, clothStartY, pinTopCorners);
        HinaPE::build_cloth_grid_soa(cloth_soa_, clothNx_, clothNy_, clothWidth, clothHeight, clothStartY, pinTopCorners);
        HinaPE::build_cloth_grid_aosoa(cloth_aosoa_, clothNx_, clothNy_, clothWidth, clothHeight, clothStartY, pinTopCorners);
        HinaPE::build_cloth_grid_aligned(cloth_aligned_, clothNx_, clothNy_, clothWidth, clothHeight, clothStartY, pinTopCorners);
    }

    void buildGeometry() {
        lineVerts_.clear();
        triVerts_.clear();

        // Fit to NDC with a uniform scale so cloth stays centered and visible
        // Positions are already around origin; we map directly to NDC in CPU.
        auto addLine = [&](float x0, float y0, float x1, float y1, float r, float g, float b, float a){
            lineVerts_.push_back(Vertex{x0, y0, r,g,b,a});
            lineVerts_.push_back(Vertex{x1, y1, r,g,b,a});
        };
        auto addQuad = [&](float cx, float cy, float size, float r, float g, float b, float a){
            float hs = size * 0.5f;
            float x0 = cx - hs, y0 = cy - hs;
            float x1 = cx + hs, y1 = cy + hs;
            // Two triangles
            triVerts_.push_back(Vertex{x0,y0,r,g,b,a});
            triVerts_.push_back(Vertex{x1,y0,r,g,b,a});
            triVerts_.push_back(Vertex{x1,y1,r,g,b,a});
            triVerts_.push_back(Vertex{x0,y0,r,g,b,a});
            triVerts_.push_back(Vertex{x1,y1,r,g,b,a});
            triVerts_.push_back(Vertex{x0,y1,r,g,b,a});
        };

        float baseSize = 0.012f;
        if (mode_ == 0) {
            for (const auto& c : cloth_aos_.constraints) {
                const auto& a = cloth_aos_.particles[c.i];
                const auto& b = cloth_aos_.particles[c.j];
                addLine(a.x, a.y, b.x, b.y, 0.2f, 0.7f, 1.0f, 0.6f);
            }
            for (const auto& p : cloth_aos_.particles) {
                float r = (p.inv_mass == 0.0f) ? 1.0f : 1.0f;
                float g = (p.inv_mass == 0.0f) ? 0.2f : 0.8f;
                float b = (p.inv_mass == 0.0f) ? 0.2f : 0.3f;
                addQuad(p.x, p.y, baseSize, r,g,b,1.0f);
            }
        } else if (mode_ == 1) {
            for (size_t k=0;k<cloth_soa_.ci.size();++k) {
                int ia = cloth_soa_.ci[k], ib = cloth_soa_.cj[k];
                addLine(cloth_soa_.x[ia], cloth_soa_.y[ia], cloth_soa_.x[ib], cloth_soa_.y[ib], 0.2f,0.7f,1.0f,0.6f);
            }
            for (size_t i=0;i<cloth_soa_.x.size();++i) {
                float invm = cloth_soa_.inv_mass[i];
                float r = (invm==0.0f)?1.0f:1.0f, g=(invm==0.0f)?0.2f:0.8f, b=(invm==0.0f)?0.2f:0.3f;
                addQuad(cloth_soa_.x[i], cloth_soa_.y[i], baseSize, r,g,b,1.0f);
            }
        } else if (mode_ == 2) {
            int nb = (cloth_aosoa_.count + HinaPE::AOSOA_BLOCK - 1)/HinaPE::AOSOA_BLOCK;
            int ncb = (cloth_aosoa_.cons_count + HinaPE::AOSOA_BLOCK - 1)/HinaPE::AOSOA_BLOCK;
            for (int cb=0; cb<ncb; ++cb) {
                const auto& blk = cloth_aosoa_.cblocks[cb];
                for (int l=0;l<HinaPE::AOSOA_BLOCK;++l) {
                    if (cb*HinaPE::AOSOA_BLOCK + l >= cloth_aosoa_.cons_count) break;
                    int ia = blk.i[l], ib = blk.j[l];
                    int bai=ia/HinaPE::AOSOA_BLOCK, lai=ia%HinaPE::AOSOA_BLOCK;
                    int bbi=ib/HinaPE::AOSOA_BLOCK, lbi=ib%HinaPE::AOSOA_BLOCK;
                    const auto& pa = cloth_aosoa_.pblocks[bai];
                    const auto& pb = cloth_aosoa_.pblocks[bbi];
                    addLine(pa.x[lai], pa.y[lai], pb.x[lbi], pb.y[lbi], 0.2f,0.7f,1.0f,0.6f);
                }
            }
            for (int b=0;b<nb;++b) {
                const auto& pb = cloth_aosoa_.pblocks[b];
                for (int l=0;l<HinaPE::AOSOA_BLOCK;++l) {
                    int idx = b*HinaPE::AOSOA_BLOCK + l; if (idx>=cloth_aosoa_.count) break;
                    float invm = pb.inv_mass[l];
                    float r = (invm==0.0f)?1.0f:1.0f, g=(invm==0.0f)?0.2f:0.8f, b=(invm==0.0f)?0.2f:0.3f;
                    addQuad(pb.x[l], pb.y[l], baseSize, r,g,b,1.0f);
                }
            }
        } else {
            // Aligned SoA
            for (size_t k=0;k<cloth_aligned_.ci.size();++k) {
                int ia = cloth_aligned_.ci[k], ib = cloth_aligned_.cj[k];
                addLine(cloth_aligned_.x[ia], cloth_aligned_.y[ia], cloth_aligned_.x[ib], cloth_aligned_.y[ib], 0.2f,0.7f,1.0f,0.6f);
            }
            for (size_t i=0;i<cloth_aligned_.x.size();++i) {
                float invm = cloth_aligned_.inv_mass[i];
                float r = (invm==0.0f)?1.0f:1.0f, g=(invm==0.0f)?0.2f:0.8f, b=(invm==0.0f)?0.2f:0.3f;
                addQuad(cloth_aligned_.x[i], cloth_aligned_.y[i], baseSize, r,g,b,1.0f);
            }
        }

        lineVertCount_ = static_cast<uint32_t>(lineVerts_.size());
        triVertCount_  = static_cast<uint32_t>(triVerts_.size());
    }

    void allocateBuffers() {
        // Initial conservative sizes across all layouts
        size_t maxConstraintCount = cloth_aos_.constraints.size();
        maxConstraintCount = std::max(maxConstraintCount, cloth_soa_.ci.size());
        maxConstraintCount = std::max(maxConstraintCount, static_cast<size_t>(cloth_aosoa_.cons_count));
        maxConstraintCount = std::max(maxConstraintCount, cloth_aligned_.ci.size());

        size_t maxParticleCount = cloth_aos_.particles.size();
        maxParticleCount = std::max(maxParticleCount, cloth_soa_.x.size());
        maxParticleCount = std::max(maxParticleCount, static_cast<size_t>(cloth_aosoa_.count));
        maxParticleCount = std::max(maxParticleCount, cloth_aligned_.x.size());

        size_t maxLines = maxConstraintCount * 2ull + 1024;
        size_t maxTris  = maxParticleCount * 6ull + 1024;
        create_buffer(allocator_, maxLines * sizeof(Vertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vboLines_);
        create_buffer(allocator_, maxTris  * sizeof(Vertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vboTris_);
    }

    void uploadGeometry() {
        // Resize if needed
        auto ensure = [&](GpuBuffer& buf, size_t needed){
            if (needed <= buf.sizeBytes) return;
            destroy_buffer(allocator_, buf);
            create_buffer(allocator_, needed + 4096, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, buf);
        };
        ensure(vboLines_, lineVerts_.size()*sizeof(Vertex));
        ensure(vboTris_,  triVerts_.size()*sizeof(Vertex));

        if (vboLines_.buffer) {
            void* ptr=nullptr; vmaMapMemory(allocator_, vboLines_.alloc, &ptr);
            std::memcpy(ptr, lineVerts_.data(), lineVerts_.size()*sizeof(Vertex));
            vmaUnmapMemory(allocator_, vboLines_.alloc);
        }
        if (vboTris_.buffer) {
            void* ptr=nullptr; vmaMapMemory(allocator_, vboTris_.alloc, &ptr);
            std::memcpy(ptr, triVerts_.data(), triVerts_.size()*sizeof(Vertex));
            vmaUnmapMemory(allocator_, vboTris_.alloc);
        }
    }

private:
    VkDevice device_{VK_NULL_HANDLE};
    VmaAllocator allocator_{VK_NULL_HANDLE};
    VkPipelineLayout pipeLayout_{VK_NULL_HANDLE};
    VkPipeline pipeLines_{VK_NULL_HANDLE};
    VkPipeline pipeTris_{VK_NULL_HANDLE};

    float viewportW_{1280.f};
    float viewportH_{720.f};
    bool simulate_{true};
    bool on_step_once_{false};
    float sim_speed_{1.0f};
    HinaPE::XPBDParams ui_params_{[](){ HinaPE::XPBDParams p{}; p.ax=0; p.ay=-9.81f; p.az=0; p.iterations=10; p.substeps=1; p.min_dt=1.f/400.f; p.max_dt=1.f/30.f; p.velocity_damping=0.01f; p.warmstart=false; p.lambda_decay=1.f; p.compliance_scale_all=1.f; p.compliance_scale_structural=1.f; p.compliance_scale_shear=1.f; p.compliance_scale_bending=1.f; p.max_correction=0.f; p.write_debug_fields=0; return p; }()};

    int mode_{0};    // 0: AoS, 1: SoA, 2: AoSoA, 3: Aligned SoA
    int backend_{0}; // 0: Native, 1: TBB, 2: AVX2
    HinaPE::ClothAOS cloth_aos_{};
    HinaPE::ClothSOA cloth_soa_{};
    HinaPE::ClothAoSoA cloth_aosoa_{};
    HinaPE::ClothAligned cloth_aligned_{};

    std::vector<Vertex> lineVerts_;
    std::vector<Vertex> triVerts_;
    GpuBuffer vboLines_;
    GpuBuffer vboTris_;
    uint32_t lineVertCount_{0};
    uint32_t triVertCount_{0};

    int clothNx_{0}, clothNy_{0};

    struct ResidOut { double l1{}, l2{}, linf{}; size_t npts{}, ncons{}; };
    ResidOut computeResidual() const {
        auto compute_aos = [&](const HinaPE::ClothAOS& c){ double s1=0,s2=0,sm=0; for (const auto& con: c.constraints){ const auto&a=c.particles[con.i]; const auto&b=c.particles[con.j]; double dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-con.rest_length; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} return ResidOut{ s1/std::max<size_t>(1,c.constraints.size()), std::sqrt(s2/std::max<size_t>(1,c.constraints.size())), sm, c.particles.size(), c.constraints.size() }; };
        auto compute_soa = [&](const HinaPE::ClothSOA& c){ double s1=0,s2=0,sm=0; for (size_t k=0;k<c.ci.size();++k){ int i=c.ci[k], j=c.cj[k]; double dx=c.x[i]-c.x[j], dy=c.y[i]-c.y[j], dz=c.z[i]-c.z[j]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-c.rest_length[k]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} return ResidOut{ s1/std::max<size_t>(1,c.ci.size()), std::sqrt(s2/std::max<size_t>(1,c.ci.size())), sm, c.x.size(), c.ci.size() }; };
        auto compute_aosoa = [&](const HinaPE::ClothAoSoA& c){ double s1=0,s2=0,sm=0; size_t m=static_cast<size_t>(c.cons_count); for (int cb=0; cb<(m+HinaPE::AOSOA_BLOCK-1)/HinaPE::AOSOA_BLOCK; ++cb){ const auto& blk=c.cblocks[cb]; for (int l=0;l<HinaPE::AOSOA_BLOCK;++l){ size_t k=static_cast<size_t>(cb*HinaPE::AOSOA_BLOCK + l); if (k>=m) break; int ia=blk.i[l], ib=blk.j[l]; int bai=ia/HinaPE::AOSOA_BLOCK,lai=ia%HinaPE::AOSOA_BLOCK; int bbi=ib/HinaPE::AOSOA_BLOCK,lbi=ib%HinaPE::AOSOA_BLOCK; const auto& pa=c.pblocks[bai]; const auto& pb=c.pblocks[bbi]; double dx=pa.x[lai]-pb.x[lbi],dy=pa.y[lai]-pb.y[lbi],dz=pa.z[lai]-pb.z[lbi]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-blk.rest_length[l]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} } return ResidOut{ s1/std::max<size_t>(1, m), std::sqrt(s2/std::max<size_t>(1, m)), sm, static_cast<size_t>(c.count), static_cast<size_t>(c.cons_count) }; };
        auto compute_aligned = [&](const HinaPE::ClothAligned& c){ double s1=0,s2=0,sm=0; size_t m=c.ci.size(); for (size_t k=0;k<m;++k){ int i=c.ci[k], j=c.cj[k]; double dx=c.x[i]-c.x[j],dy=c.y[i]-c.y[j],dz=c.z[i]-c.z[j]; double r=std::sqrt(dx*dx+dy*dy+dz*dz)-c.rest_length[k]; double v=std::abs(r); s1+=v; s2+=r*r; sm=std::max(sm,v);} return ResidOut{ s1/std::max<size_t>(1, m), std::sqrt(s2/std::max<size_t>(1, m)), sm, c.x.size(), c.ci.size() }; };
        if (mode_==0) return compute_aos(cloth_aos_);
        if (mode_==1) return compute_soa(cloth_soa_);
        if (mode_==2) return compute_aosoa(cloth_aosoa_);
        return compute_aligned(cloth_aligned_);
    }
};

int main() {
    try {
        VulkanEngine engine;
        engine.state_.name = "XPBD Cloth (Vulkan Visualizer)";
        engine.state_.width = 1280;
        engine.state_.height = 720;
        engine.set_renderer(std::make_unique<XPBDRenderer>());
        engine.init();
        engine.run();
        engine.cleanup();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal error: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

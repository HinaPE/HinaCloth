#include "vk_engine.h"
#include <vulkan/vulkan.h>

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <algorithm>

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
#include "aos/solver_xpbd_aos.h"
#include "soa/solver_xpbd_soa.h"
#include "aosoa/solver_xpbd_aosoa.h"

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
        }
    }

    void update(const EngineContext&, const FrameContext& frm) override {
        const float dt = (float)frm.dt_sec;
        HinaPE::XPBDParams params;
        params.ax = 0.0f; params.ay = -9.81f; params.az = 0.0f;
        params.iterations = 10;
        params.substeps = 1;
        params.min_dt = 1.0f/400.0f;
        params.max_dt = 1.0f/30.0f;
        params.velocity_damping = 0.01f;
        params.warmstart = false;
        params.lambda_decay = 1.0f;
        params.compliance_scale_all = 1.0f;
        params.compliance_scale_structural = 1.0f;
        params.compliance_scale_shear = 1.0f;
        params.compliance_scale_bending = 1.0f;
        params.max_correction = 0.0f;
        params.write_debug_fields = 0;
        if (mode_ == 0) HinaPE::xpbd_step_aos(cloth_aos_, dt, params);
        else if (mode_ == 1) HinaPE::xpbd_step_soa(cloth_soa_, dt, params);
        else HinaPE::xpbd_step_aosoa(cloth_aosoa_, dt, params);
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
        HinaPE::build_cloth_grid_aos(cloth_aos_, 40, 25, 1.6f, 1.0f, +0.3f, true);
        HinaPE::build_cloth_grid_soa(cloth_soa_, 40, 25, 1.6f, 1.0f, +0.3f, true);
        HinaPE::build_cloth_grid_aosoa(cloth_aosoa_, 40, 25, 1.6f, 1.0f, +0.3f, true);
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
        } else {
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
        }

        lineVertCount_ = static_cast<uint32_t>(lineVerts_.size());
        triVertCount_  = static_cast<uint32_t>(triVerts_.size());
    }

    void allocateBuffers() {
        // Initial conservative sizes across all layouts
        size_t maxLines = std::max({ cloth_aos_.constraints.size()*2ull,
                                     cloth_soa_.ci.size()*2ull,
                                     (size_t)cloth_aosoa_.cons_count*2ull }) + 1024;
        size_t maxTris  = std::max({ cloth_aos_.particles.size()*6ull,
                                     cloth_soa_.x.size()*6ull,
                                     (size_t)cloth_aosoa_.count*6ull }) + 1024;
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

    int mode_{0}; // 0: AoS, 1: SoA, 2: AoSoA
    HinaPE::ClothAOS cloth_aos_{};
    HinaPE::ClothSOA cloth_soa_{};
    HinaPE::ClothAoSoA cloth_aosoa_{};

    std::vector<Vertex> lineVerts_;
    std::vector<Vertex> triVerts_;
    GpuBuffer vboLines_;
    GpuBuffer vboTris_;
    uint32_t lineVertCount_{0};
    uint32_t triVertCount_{0};

    int clothNx_{0}, clothNy_{0};
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

// XPBD Cloth Visualization with VulkanVisualizer
// - Renders points and constraints using Vulkan pipelines (dynamic rendering)
// - Uses HinaPE::ClothData + xpbd_step_native for simulation
// - Rich ImGui control panel for parameters

#include "vk_engine.h"
#include "xpbd.h"
#include "xpbd2.h"
#include "cloth_data_2.h"

#include <SDL3/SDL.h>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using HinaPE::XPBDParams;
using HinaPE::u32;

#ifndef VK_CHECK
#define VK_CHECK(x) do { VkResult _res=(x); if(_res!=VK_SUCCESS){ std::fprintf(stderr,"Vulkan error %d at %s\n", (int)_res, #x); std::abort(); } } while(false)
#endif

struct float2 { float x{0}, y{0}; };

struct Camera2D {
    float2 center{0.0f, 0.0f};
    float  pixelsPerUnit{120.0f}; // pixels per world-unit
};

struct PushConstants {
    // NDC transform: x' = a.x*x + b.x; y' = a.y*y + b.y
    float ax, ay, bx, by;
    float r, g, b, a;     // color
    float pointSize;      // for point list (pixels)
};

class XpbdClothRenderer final : public IRenderer {
public:
    void initialize(const EngineContext&) override { rebuildCloth(); }
    void destroy(const EngineContext&) override {}
    void get_capabilities(RendererCaps& caps) const override { caps = RendererCaps{}; }
    void on_swapchain_ready(const EngineContext&, const FrameContext& frm) override {
        viewport_ = ImVec2((float)frm.extent.width, (float)frm.extent.height);
        createPipelinesRequested_ = true;
    }
    void on_swapchain_destroy(const EngineContext&) override {}

    void update(const EngineContext&, const FrameContext& frm) override {
        // Apply any rebuild requested in previous frame's UI
        if (pendingRebuild_) { rebuildCloth(); pendingRebuild_ = false; }

        if (!sim_.paused) {
            double dt = frm.dt_sec * sim_.timeScale;
            accumulator_ += dt;
            const double maxFrame = 0.05; // clamp long frames
            if (accumulator_ > 0.25) accumulator_ = 0.25;
            double step = std::max(1e-6, (double)sim_.dt);
            int maxSteps = (int)std::ceil(std::min(accumulator_, maxFrame) / step);
            int steps = std::min(maxSteps, 16);
            for (int s = 0; s < steps; ++s) {
                XPBDParams p{};
                p.time_step = sim_.dt;
                p.substeps = sim_.substeps;
                p.solver_iterations = sim_.iterations;
                p.gravity_x = sim_.gravity.x; p.gravity_y = sim_.gravity.y; p.gravity_z = 0.0f;
                p.velocity_damping = sim_.damping;
                p.enable_distance_constraints = true;
                p.reset_hard_lambda_each_substep = sim_.resetHardEachSubstep;
                p.use_color_ordering = sim_.useColorOrdering;
                // propagate compliance to all edges
                for (size_t i=0;i<cloth2_.numEdges();++i) cloth2_.compliance[i] = sim_.compliance;
                xpbd_step_native2(cloth2_, p);
                accumulator_ -= step;
            }
            if (accumulator_ < 0.0) accumulator_ = 0.0;
        }
        // Update GPU vertex data
        updateGPUDataRequested_ = true;
    }

    void on_imgui(const EngineContext&, const FrameContext& frm) override {
        ImGui::SetNextWindowPos(ImVec2(16, 16), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(420, 520), ImGuiCond_Once);
        ImGui::Begin("XPBD Cloth");
        ImGui::Text("FPS: %.1f", 1.0f/std::max(1e-6f, (float)frm.dt_sec));
        ImGui::Checkbox("Paused", &sim_.paused);
        ImGui::SliderFloat("Time Scale", &sim_.timeScale, 0.0f, 2.0f, "%.2f");
        ImGui::SliderFloat("dt (s)", &sim_.dt, 1e-4f, 0.033f, "%.5f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderInt("Substeps", &sim_.substeps, 1, 8);
        ImGui::SliderInt("Iterations", &sim_.iterations, 1, 80);
        ImGui::SliderFloat("Damping", &sim_.damping, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Compliance", &sim_.compliance, 0.0f, 1e-2f, "%.6f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat2("Gravity", &sim_.gravity.x, -50.0f, 50.0f, "%.2f");
        ImGui::SeparatorText("XPBD Options");
        ImGui::Checkbox("Use Color Ordering", &sim_.useColorOrdering);
        ImGui::Checkbox("Reset Hard Lambda Each Substep", &sim_.resetHardEachSubstep);
        ImGui::SeparatorText("Rendering");
        ImGui::Checkbox("Show Edges", &sim_.showEdges); ImGui::SameLine(); ImGui::Checkbox("Show Points", &sim_.showPoints);
        ImGui::SliderFloat("Point Size (px)", &sim_.pointSize, 1.0f, 10.0f, "%.1f");
        ImGui::ColorEdit3("Edge Color", &sim_.edgeColor.x);
        ImGui::ColorEdit3("Point Color", &sim_.ptColor.x);
        ImGui::SeparatorText("Cloth");
        bool rebuild=false;
        rebuild |= ImGui::SliderInt("Width", &sim_.W, 2, 200);
        rebuild |= ImGui::SliderInt("Height", &sim_.H, 2, 200);
        rebuild |= ImGui::SliderFloat("Spacing", &sim_.spacing, 0.01f, 0.5f, "%.3f");
        ImGui::Checkbox("Pin Left", &sim_.pinL); ImGui::SameLine(); ImGui::Checkbox("Pin Right", &sim_.pinR);
        if (ImGui::Button("Reset Camera")) { cam_ = Camera2D{}; fitView(); }
        ImGui::SameLine(); if (ImGui::Button("Fit View")) fitView();
        if (rebuild) pendingRebuild_ = true;
        if (ImGui::Button("Rebuild")) { rebuildCloth(); pendingRebuild_ = false; }
        ImGui::End();
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) override {
        if (createPipelinesRequested_) {
            destroyPipelines(eng.device);
            createPipelines(eng.device, frm);
            createPipelinesRequested_ = false;
        }
        if (updateGPUDataRequested_) {
            ensureBuffers(eng);
            uploadVertexData();
            updateGPUDataRequested_ = false;
        }

        // Begin dynamic rendering into offscreen drawable (R16G16B16A16_SFLOAT)
        VkRenderingAttachmentInfo color_attach{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color_attach.imageView = frm.offscreen_image_view;
        color_attach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        color_attach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        VkClearValue clear; clear.color = {{0.08f, 0.08f, 0.09f, 1.0f}};
        color_attach.clearValue = clear;
        color_attach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

        VkRenderingInfo render_info{VK_STRUCTURE_TYPE_RENDERING_INFO};
        render_info.renderArea = {{0,0}, frm.extent};
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 1;
        render_info.pColorAttachments = &color_attach;
        vkCmdBeginRendering(cmd, &render_info);

        // Viewport/scissor
        VkViewport vp{}; vp.x = 0; vp.y = 0; vp.width = (float)frm.extent.width; vp.height = (float)frm.extent.height; vp.minDepth = 0.0f; vp.maxDepth = 1.0f;
        VkRect2D sc{}; sc.offset = {0,0}; sc.extent = frm.extent;
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);

        // Compute world->NDC transform
        // Flip Y so that simulation +Y up appears up on screen (Vulkan framebuffer Y grows down).
        const float sx = 2.0f / (frm.extent.width / std::max(1e-6f, cam_.pixelsPerUnit));
        const float sy = -2.0f / (frm.extent.height / std::max(1e-6f, cam_.pixelsPerUnit));
        const float bx = -cam_.center.x * sx;
        const float by = -cam_.center.y * sy;

        // Draw constraints (lines)
        if (pipeLines_ && sim_.showEdges) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLines_);
            VkDeviceSize offs = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &vbLines_, &offs);
            PushConstants pc{sx, sy, bx, by, sim_.edgeColor.x, sim_.edgeColor.y, sim_.edgeColor.z, 1.0f, 1.0f};
            vkCmdPushConstants(cmd, pipeLayout_, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants), &pc);
            vkCmdDraw(cmd, linesCount_, 1, 0, 0);
        }

        // Draw points
        if (pipePoints_ && sim_.showPoints) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipePoints_);
            VkDeviceSize offs = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &vbPoints_, &offs);
            PushConstants pc{sx, sy, bx, by, sim_.ptColor.x, sim_.ptColor.y, sim_.ptColor.z, 1.0f, sim_.pointSize};
            vkCmdPushConstants(cmd, pipeLayout_, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants), &pc);
            vkCmdDraw(cmd, pointsCount_, 1, 0, 0);
        }

        vkCmdEndRendering(cmd);
    }

private:
    void rebuildCloth() {
        const int W = std::max(2, sim_.W);
        const int H = std::max(2, sim_.H);
        const size_t N = (size_t)W * (size_t)H;
        cloth2_.allocateParticles(N);
        float halfW = 0.5f * (W - 1) * sim_.spacing;
        float halfH = 0.5f * (H - 1) * sim_.spacing;
        float2 origin{cam_.center.x - halfW, cam_.center.y + halfH};
        for (int y=0;y<H;++y){
            for (int x=0;x<W;++x){
                size_t id = (size_t)y * (size_t)W + (size_t)x;
                cloth2_.px[id] = origin.x + x*sim_.spacing;
                cloth2_.py[id] = origin.y - y*sim_.spacing;
                cloth2_.pz[id] = 0.0f;
                cloth2_.vx[id] = 0.0f; cloth2_.vy[id] = 0.0f; cloth2_.vz[id] = 0.0f;
                cloth2_.inv_mass[id] = 1.0f; cloth2_.pinned[id] = 0;
            }
        }
        // edges
        std::vector<u32> ei, ej; std::vector<float> rest, comp, lam, alp; std::vector<HinaPE::u8> cols;
        auto addEdge=[&](int x0,int y0,int x1,int y1){
            size_t i=(size_t)y0*W + (size_t)x0; size_t j=(size_t)y1*W + (size_t)x1;
            float dx=cloth2_.px[i]-cloth2_.px[j]; float dy=cloth2_.py[i]-cloth2_.py[j];
            float r=std::sqrt(dx*dx+dy*dy);
            ei.push_back((u32)i); ej.push_back((u32)j); rest.push_back(r);
            comp.push_back(sim_.compliance); lam.push_back(0.0f); alp.push_back(0.0f);
            HinaPE::u8 color = (y0==y1) ? HinaPE::u8((y0 & 1)) : HinaPE::u8(2 + (x0 & 1));
            cols.push_back(color);
        };
        for(int y=0;y<H;++y){ for(int x=0;x<W;++x){ if(x+1<W) addEdge(x,y,x+1,y); if(y+1<H) addEdge(x,y,x,y+1);} }
        cloth2_.allocateDistance(ei.size());
        for (size_t k=0;k<ei.size();++k) {
            cloth2_.edge_i[k] = ei[k];
            cloth2_.edge_j[k] = ej[k];
            cloth2_.rest[k]   = rest[k];
            cloth2_.compliance[k] = comp[k];
            cloth2_.lambda[k] = lam[k];
            cloth2_.alpha[k]  = alp[k];
            cloth2_.color[k]  = cols[k];
        }

        // pin
        cloth2_.pinned[0] = sim_.pinL ? 1u : 0u; cloth2_.inv_mass[0] = sim_.pinL ? 0.0f : 1.0f;
        size_t right = (size_t)W - 1;
        cloth2_.pinned[right] = sim_.pinR ? 1u : 0u; cloth2_.inv_mass[right] = sim_.pinR ? 0.0f : 1.0f;

        fitView();
        createPipelinesRequested_ = true;
        updateGPUDataRequested_ = true;
    }

    void fitView() {
        // Center cloth and choose pixelsPerUnit such that it fits view
        const size_t n = cloth2_.numParticles();
        if (n == 0) return;
        float minx = cloth2_.px[0], maxx = cloth2_.px[0];
        float miny = cloth2_.py[0], maxy = cloth2_.py[0];
        for (size_t i=1;i<n;++i){
            minx = std::min(minx, cloth2_.px[i]); maxx = std::max(maxx, cloth2_.px[i]);
            miny = std::min(miny, cloth2_.py[i]); maxy = std::max(maxy, cloth2_.py[i]);
        }
        cam_.center = { (minx+maxx)*0.5f, (miny+maxy)*0.5f };
        float wx = std::max(1e-6f, maxx-minx);
        float wy = std::max(1e-6f, maxy-miny);
        float sx = (viewport_.x*0.9f) / wx;
        float sy = (viewport_.y*0.9f) / wy;
        cam_.pixelsPerUnit = std::max(30.0f, std::min(sx, sy));
    }

private:
    struct SimUI {
        bool paused{false};
        float dt{1.0f/120.0f};
        int substeps{1};
        int iterations{10};
        float damping{0.01f};
        float compliance{1e-9f};
        float2 gravity{0.0f,-9.81f};
        int W{32}; int H{32}; float spacing{0.03f};
        bool pinL{true}; bool pinR{true};
        float timeScale{1.0f};
        // XPBD options
        bool resetHardEachSubstep{false};
        bool useColorOrdering{true};
        // Rendering options
        bool showPoints{true};
        bool showEdges{true};
        float pointSize{4.0f};
        ImVec4 edgeColor{0.55f, 0.75f, 1.0f, 1.0f};
        ImVec4 ptColor{1.0f, 0.94f, 0.47f, 1.0f};
    } sim_{};

    HinaPE::ClothData2 cloth2_{};
    Camera2D cam_{};
    ImVec2 viewport_{1280.0f, 720.0f};
    double accumulator_{0.0};
    bool pendingRebuild_{false};

    // GPU resources
    VkPipelineLayout pipeLayout_{VK_NULL_HANDLE};
    VkPipeline pipePoints_{VK_NULL_HANDLE};
    VkPipeline pipeLines_{VK_NULL_HANDLE};
    VkFormat colorFormat_{VK_FORMAT_R16G16B16A16_SFLOAT};

    VkBuffer vbPoints_{VK_NULL_HANDLE}; VmaAllocation vbPointsAlloc_{}; void* vbPointsMapped_{nullptr}; uint32_t pointsCapacity_{0}; uint32_t pointsCount_{0};
    VkBuffer vbLines_{VK_NULL_HANDLE};  VmaAllocation vbLinesAlloc_{};  void* vbLinesMapped_{nullptr};  uint32_t linesCapacity_{0};  uint32_t linesCount_{0};

    bool createPipelinesRequested_{false};
    bool updateGPUDataRequested_{false};

    void createPipelines(VkDevice device, const FrameContext& frm) {
        // Shader modules
        auto read_file = [](const char* path){ std::vector<char> data; std::ifstream f(path, std::ios::binary); if(!f) return data; f.seekg(0,std::ios::end); size_t sz=(size_t)f.tellg(); f.seekg(0); data.resize(sz); f.read(data.data(), sz); return data; };
        std::string vsPath = std::string(SHADER_OUTPUT_DIR) + "/xpbd.vert.spv";
        std::string fsPath = std::string(SHADER_OUTPUT_DIR) + "/xpbd.frag.spv";
        auto vsCode = read_file(vsPath.c_str());
        auto fsCode = read_file(fsPath.c_str());
        VkShaderModuleCreateInfo smci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        smci.codeSize = vsCode.size(); smci.pCode = vsCode.empty()? nullptr : reinterpret_cast<const uint32_t*>(vsCode.data());
        VkShaderModule vs{}; VK_CHECK(vkCreateShaderModule(device, &smci, nullptr, &vs));
        smci.codeSize = fsCode.size(); smci.pCode = fsCode.empty()? nullptr : reinterpret_cast<const uint32_t*>(fsCode.data());
        VkShaderModule fs{}; VK_CHECK(vkCreateShaderModule(device, &smci, nullptr, &fs));

        // Pipeline layout (push constants only)
        VkPushConstantRange pcr{}; pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT; pcr.offset = 0; pcr.size = sizeof(PushConstants);
        VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; plci.pushConstantRangeCount = 1; plci.pPushConstantRanges = &pcr;
        VK_CHECK(vkCreatePipelineLayout(device, &plci, nullptr, &pipeLayout_));

        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT; stages[0].module = vs; stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module = fs; stages[1].pName = "main";

        VkVertexInputBindingDescription bind{0, sizeof(float2), VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0, 0, VK_FORMAT_R32G32_SFLOAT, 0};
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vi.vertexBindingDescriptionCount = 1; vi.pVertexBindingDescriptions = &bind; vi.vertexAttributeDescriptionCount = 1; vi.pVertexAttributeDescriptions = &attr;

        auto create_pipeline = [&](VkPrimitiveTopology topo){
            VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology = topo; ia.primitiveRestartEnable = VK_FALSE;
            VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount = 1; vp.scissorCount = 1;
            VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rs.polygonMode = VK_POLYGON_MODE_FILL; rs.cullMode = VK_CULL_MODE_NONE; rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth = 1.0f;
            VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
            VkPipelineColorBlendAttachmentState cbatt{}; cbatt.colorWriteMask = 0xF;
            VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount = 1; cb.pAttachments = &cbatt;
            VkDynamicState dynArr[2]{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; dyn.dynamicStateCount = 2; dyn.pDynamicStates = dynArr;

            VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; VkFormat fmt = colorFormat_; rendering.colorAttachmentCount = 1; rendering.pColorAttachmentFormats = &fmt;
            VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            gp.pNext = &rendering; gp.stageCount = 2; gp.pStages = stages; gp.pVertexInputState = &vi; gp.pInputAssemblyState = &ia; gp.pViewportState = &vp; gp.pRasterizationState = &rs; gp.pMultisampleState = &ms; gp.pColorBlendState = &cb; gp.pDynamicState = &dyn; gp.layout = pipeLayout_;
            VkPipeline pipe{}; VK_CHECK(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gp, nullptr, &pipe));
            return pipe;
        };

        pipePoints_ = create_pipeline(VK_PRIMITIVE_TOPOLOGY_POINT_LIST);
        pipeLines_  = create_pipeline(VK_PRIMITIVE_TOPOLOGY_LINE_LIST);

        vkDestroyShaderModule(device, vs, nullptr);
        vkDestroyShaderModule(device, fs, nullptr);
    }

    void destroyPipelines(VkDevice device) {
        if (pipePoints_ && sim_.showPoints) { vkDestroyPipeline(device, pipePoints_, nullptr); pipePoints_ = VK_NULL_HANDLE; }
        if (pipeLines_)  { vkDestroyPipeline(device, pipeLines_,  nullptr); pipeLines_  = VK_NULL_HANDLE; }
        if (pipeLayout_) { vkDestroyPipelineLayout(device, pipeLayout_, nullptr); pipeLayout_ = VK_NULL_HANDLE; }
    }

    void ensureBuffers(const EngineContext& eng) {
        uint32_t needPts = (uint32_t)cloth2_.numParticles();
        uint32_t needLines = (uint32_t)cloth2_.numEdges() * 2u;
        if (needPts > pointsCapacity_) { destroyBuffer(eng.allocator, vbPoints_, vbPointsAlloc_); createBuffer(eng.allocator, needPts * sizeof(float2), vbPoints_, vbPointsAlloc_, &vbPointsMapped_); pointsCapacity_ = needPts; }
        if (needLines > linesCapacity_) { destroyBuffer(eng.allocator, vbLines_, vbLinesAlloc_); createBuffer(eng.allocator, needLines * sizeof(float2), vbLines_, vbLinesAlloc_, &vbLinesMapped_); linesCapacity_ = needLines; }
    }

    void createBuffer(VmaAllocator allocator, VkDeviceSize size, VkBuffer& buf, VmaAllocation& alloc, void** mapped) {
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bci.size = size; bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT; bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_AUTO; aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo ai{}; VK_CHECK(vmaCreateBuffer(allocator, &bci, &aci, &buf, &alloc, &ai)); *mapped = ai.pMappedData;
    }

    void destroyBuffer(VmaAllocator allocator, VkBuffer& buf, VmaAllocation& alloc) {
        if (buf != VK_NULL_HANDLE) { vmaDestroyBuffer(allocator, buf, alloc); buf = VK_NULL_HANDLE; alloc = nullptr; }
    }

    void uploadVertexData() {
        // points
        pointsCount_ = (uint32_t)cloth2_.numParticles();
        if (vbPointsMapped_ && pointsCount_) {
            auto* dst = reinterpret_cast<float2*>(vbPointsMapped_);
            for (uint32_t i=0;i<pointsCount_;++i) dst[i] = float2{cloth2_.px[i], cloth2_.py[i]};
        }
        // lines
        linesCount_ = (uint32_t)cloth2_.numEdges() * 2u;
        if (vbLinesMapped_ && linesCount_) {
            auto* dst = reinterpret_cast<float2*>(vbLinesMapped_);
            uint32_t k = 0;
            for (uint32_t c=0;c<(uint32_t)cloth2_.numEdges();++c){ u32 i=cloth2_.edge_i[c], j=cloth2_.edge_j[c]; dst[k++] = {cloth2_.px[i], cloth2_.py[i]}; dst[k++] = {cloth2_.px[j], cloth2_.py[j]}; }
        }
    }
};

int main() {
    try {
        VulkanEngine engine;
        engine.state_.name = "XPBD Cloth (Vulkan)";
        engine.state_.width = 1600; engine.state_.height = 900;
        engine.set_renderer(std::make_unique<XpbdClothRenderer>());
        engine.init();
        engine.run();
        engine.cleanup();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal error: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

#include "vk_engine.h"
#include "api/sim.h"
#include <imgui.h>
#include <vulkan/vulkan.h>
#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cstring>

#ifndef VK_CHECK
#define VK_CHECK(x) do { VkResult _res = (x); if (_res != VK_SUCCESS) throw std::runtime_error(std::string("Vulkan error: ") + std::to_string(_res)); } while(false)
#endif
#ifndef SHADER_OUTPUT_DIR
#define SHADER_OUTPUT_DIR "examples/shader"
#endif

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
    edges.clear(); edges.reserve((nx*(ny-1) + ny*(nx-1)) * 2u);
    for(uint32_t j=0;j<ny;++j){ for(uint32_t i=0;i+1<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i+1,j,nx); edges.push_back(a); edges.push_back(b);} }
    for(uint32_t j=0;j+1<ny;++j){ for(uint32_t i=0;i<nx;++i){ uint32_t a=vid(i,j,nx), b=vid(i,j+1,nx); edges.push_back(a); edges.push_back(b);} }
}

static std::vector<char> load_binary(const std::string& path){
    std::ifstream f(path, std::ios::binary | std::ios::ate); if(!f) throw std::runtime_error("open fail: "+path);
    auto sz = f.tellg(); f.seekg(0); std::vector<char> buf(static_cast<size_t>(sz)); f.read(buf.data(), sz); if(!f) throw std::runtime_error("read fail: "+path); return buf;
}
static VkShaderModule make_shader(VkDevice dev, const std::vector<char>& bytes){
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO}; ci.codeSize = bytes.size(); ci.pCode = reinterpret_cast<const uint32_t*>(bytes.data());
    VkShaderModule m{}; if(vkCreateShaderModule(dev,&ci,nullptr,&m)!=VK_SUCCESS) throw std::runtime_error("shader module create failed"); return m;
}

struct AllocatedBuffer { VkBuffer buffer{}; VmaAllocation allocation{}; VkDeviceSize size{}; };
static void create_host_buffer(const EngineContext& eng, VkDeviceSize size, VkBufferUsageFlags usage, AllocatedBuffer& out){
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bi.size=size; bi.usage=usage; bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo ai{}; ai.usage = VMA_MEMORY_USAGE_AUTO; ai.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VK_CHECK(vmaCreateBuffer(eng.allocator, &bi, &ai, &out.buffer, &out.allocation, nullptr)); out.size=size;
}
static void destroy_buffer(const EngineContext& eng, AllocatedBuffer& b){ if(b.buffer){ vmaDestroyBuffer(eng.allocator, b.buffer, b.allocation); b = {}; } }

} // namespace

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

    void initialize(const EngineContext& eng, const RendererCaps& caps, const FrameContext&) override {
        device_       = eng.device;
        color_format_ = caps.color_attachments.front().format;
        // Build cloth & solver
        uint32_t nx=24, ny=16; float dx=0.05f;
        pos_.clear(); vel_.clear(); edges_.clear(); make_grid(nx,ny,dx,pos_,vel_,edges_);
        FieldView fpos{"position", FieldType::F32, pos_.data(), nx*ny, 3, sizeof(float)*3};
        FieldView fvel{"velocity", FieldType::F32, vel_.data(), nx*ny, 3, sizeof(float)*3};
        FieldView fields[2] = {fpos,fvel}; StateInit st{fields,2};
        RelationView rel{edges_.data(), 2, edges_.size()/2, "edges"}; TopologyIn topo{nx*ny, &rel, 1};
        const char* tags[] = {"edges"}; FieldUse uses[] = {{"position", true}};
        OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true}; OperatorsDecl ops{&op, 1};
        Param gy{"gravity_y", ParamType::F32, {.f32=-9.8f}}; Parameters params{&gy, 1};
        Policy pol{{DataLayout::Auto, Backend::Native, -1, true, true}, {2, 10, 0.02f, TimeStepper::Symplectic}};
        SpaceDesc sp{SpaceType::Lagrangian, 1, 0}; EventsScript ev{nullptr, 0};
        BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
        auto r = create(bd); solver_ = (r.status==Status::Ok) ? r.value : nullptr;

        // GPU: pipeline for line rendering
        const std::string dir = std::string(SHADER_OUTPUT_DIR);
        VkShaderModule vert = make_shader(device_, load_binary(dir+"/cloth_lines.vert.spv"));
        VkShaderModule frag = make_shader(device_, load_binary(dir+"/cloth_lines.frag.spv"));

        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[0].stage=VK_SHADER_STAGE_VERTEX_BIT; stages[0].module=vert; stages[0].pName="main";
        stages[1].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module=frag; stages[1].pName="main";

        VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push)};
        VkPipelineLayoutCreateInfo lci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO}; lci.pushConstantRangeCount=1; lci.pPushConstantRanges=&pcr;
        VK_CHECK(vkCreatePipelineLayout(device_, &lci, nullptr, &layout_));

        VkVertexInputBindingDescription bind{0, sizeof(float)*3, VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0};
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO}; vi.vertexBindingDescriptionCount=1; vi.pVertexBindingDescriptions=&bind; vi.vertexAttributeDescriptionCount=1; vi.pVertexAttributeDescriptions=&attr;
        VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology=VK_PRIMITIVE_TOPOLOGY_LINE_LIST; ia.primitiveRestartEnable=VK_FALSE;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount=1; vp.scissorCount=1;
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rs.polygonMode=VK_POLYGON_MODE_FILL; rs.cullMode=VK_CULL_MODE_NONE; rs.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth=1.0f;
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState ba{}; ba.colorWriteMask=VK_COLOR_COMPONENT_R_BIT|VK_COLOR_COMPONENT_G_BIT|VK_COLOR_COMPONENT_B_BIT|VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount=1; cb.pAttachments=&ba;
        const VkDynamicState dyn_states[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; dyn.dynamicStateCount=2; dyn.pDynamicStates=dyn_states;
        VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; rendering.colorAttachmentCount=1; rendering.pColorAttachmentFormats=&color_format_;
        VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pci.pNext=&rendering; pci.stageCount=2; pci.pStages=stages; pci.pVertexInputState=&vi; pci.pInputAssemblyState=&ia; pci.pViewportState=&vp; pci.pRasterizationState=&rs; pci.pMultisampleState=&ms; pci.pColorBlendState=&cb; pci.pDynamicState=&dyn; pci.layout=layout_;
        VK_CHECK(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pci, nullptr, &pipeline_));
        vkDestroyShaderModule(device_, vert, nullptr); vkDestroyShaderModule(device_, frag, nullptr);

        // Buffers: vertices (dynamic) and indices (static lines)
        create_host_buffer(eng, sizeof(float)*3u*pos_.size()/3u, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vbuf_);
        create_host_buffer(eng, sizeof(uint32_t)*edges_.size(), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, ibuf_);
        // upload indices
        void* mapped=nullptr; vmaMapMemory(eng.allocator, ibuf_.allocation, &mapped); std::memcpy(mapped, edges_.data(), sizeof(uint32_t)*edges_.size()); vmaUnmapMemory(eng.allocator, ibuf_.allocation);
    }

    void destroy(const EngineContext& eng, const RendererCaps&) override {
        destroy_buffer(eng, vbuf_); destroy_buffer(eng, ibuf_);
        if (pipeline_) vkDestroyPipeline(eng.device, pipeline_, nullptr);
        if (layout_)   vkDestroyPipelineLayout(eng.device, layout_, nullptr);
        pipeline_ = VK_NULL_HANDLE; layout_ = VK_NULL_HANDLE; device_ = VK_NULL_HANDLE;
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
        ImGui::Begin("HinaCloth Overview");
        ImGui::Text("step_ms: %.3f", tf.step_ms);
        ImGui::Text("residual_avg: %.6f", tf.residual_avg);
        ImGui::Text("substeps: %d", tf.solve_substeps);
        ImGui::Text("iterations: %d", tf.solve_iterations);
        ImGui::End();
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) override {
        if (!pipeline_ || frm.color_attachments.empty()) return;
        const AttachmentView& target = frm.color_attachments.front();

        size_t nverts = pos_.size()/3u; staging_.resize(pos_.size());
        (void)copy_positions(solver_, staging_.data(), nverts, nullptr);

        float minx=1e9f, minz=1e9f, maxx=-1e9f, maxz=-1e9f;
        for (size_t i=0;i<nverts;i++){ float x=staging_[3*i+0], z=staging_[3*i+2]; minx=std::min(minx,x); maxx=std::max(maxx,x); minz=std::min(minz,z); maxz=std::max(maxz,z);}
        float cx = 0.5f*(minx+maxx); float cz = 0.5f*(minz+maxz);
        float hx = std::max(0.5f*(maxx-minx), 1e-6f); float hz = std::max(0.5f*(maxz-minz), 1e-6f);
        float h = std::max(hx, hz);
        float aspect = frm.extent.height > 0 ? (float)frm.extent.width / (float)frm.extent.height : 1.0f;
        Push push{}; float s = 0.9f / h; push.scale[0]=s / aspect; push.scale[1]=s; push.offset[0] = -cx * push.scale[0]; push.offset[1] = -cz * push.scale[1];

        void* mapped=nullptr; vmaMapMemory(eng.allocator, vbuf_.allocation, &mapped); std::memcpy(mapped, staging_.data(), sizeof(float)*staging_.size()); vmaUnmapMemory(eng.allocator, vbuf_.allocation);

        auto barrier = [&](VkImageLayout oldL, VkImageLayout newL, VkPipelineStageFlags2 srcStage, VkPipelineStageFlags2 dstStage, VkAccessFlags2 srcAccess, VkAccessFlags2 dstAccess){
            VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2}; b.srcStageMask=srcStage; b.dstStageMask=dstStage; b.srcAccessMask=srcAccess; b.dstAccessMask=dstAccess; b.oldLayout=oldL; b.newLayout=newL; b.image=target.image; b.subresourceRange={target.aspect,0u,1u,0u,1u}; VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO}; di.imageMemoryBarrierCount=1; di.pImageMemoryBarriers=&b; vkCmdPipelineBarrier2(cmd,&di);
        };
        barrier(VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_MEMORY_WRITE_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkClearValue clear{.color={{0.06f,0.07f,0.09f,1.0f}}};
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color.imageView = target.view; color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; color.storeOp = VK_ATTACHMENT_STORE_OP_STORE; color.clearValue = clear;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea = {{0,0}, frm.extent}; ri.layerCount = 1; ri.colorAttachmentCount = 1; ri.pColorAttachments = &color;
        vkCmdBeginRendering(cmd, &ri);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
        VkViewport vp{}; vp.x=0; vp.y=0; vp.width = float(frm.extent.width); vp.height = float(frm.extent.height); vp.minDepth=0.f; vp.maxDepth=1.f;
        VkRect2D sc{{0,0}, frm.extent};
        vkCmdSetViewport(cmd,0,1,&vp); vkCmdSetScissor(cmd,0,1,&sc);
        vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push), &push);
        VkDeviceSize off=0; VkBuffer vb = vbuf_.buffer; vkCmdBindVertexBuffers(cmd,0,1,&vb,&off); vkCmdBindIndexBuffer(cmd, ibuf_.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, (uint32_t)edges_.size(), 1, 0, 0, 0);
        vkCmdEndRendering(cmd);

        barrier(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    }

private:
    struct Push { float scale[2]; float offset[2]; };

    VkDevice device_{VK_NULL_HANDLE};
    VkPipelineLayout layout_{VK_NULL_HANDLE};
    VkPipeline pipeline_{VK_NULL_HANDLE};
    VkFormat color_format_{VK_FORMAT_B8G8R8A8_UNORM};

    sim::Solver* solver_{nullptr};
    std::vector<float> pos_, vel_;
    std::vector<uint32_t> edges_;
    std::vector<float> staging_;
    AllocatedBuffer vbuf_{}; // vertex positions
    AllocatedBuffer ibuf_{}; // indices (lines)
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

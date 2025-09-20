#include "vk_engine.h"
#include <cmath>
#include <cstdio>
#include <memory>

class SimpleClearRenderer final : public IRenderer {
public:
    void get_capabilities(const EngineContext&, RendererCaps& caps) override {
        caps = RendererCaps{};
        caps.presentation_mode          = PresentationMode::EngineBlit;
        caps.preferred_swapchain_format = VK_FORMAT_B8G8R8A8_UNORM;
        caps.color_attachments = { AttachmentRequest{ .name = "color", .format = VK_FORMAT_B8G8R8A8_UNORM, .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, .samples = VK_SAMPLE_COUNT_1_BIT, .aspect = VK_IMAGE_ASPECT_COLOR_BIT, .initial_layout = VK_IMAGE_LAYOUT_GENERAL } };
        caps.presentation_attachment = "color";
        caps.enable_imgui = true;
    }

    void initialize(const EngineContext&, const RendererCaps&, const FrameContext&) override {}
    void destroy(const EngineContext&, const RendererCaps&) override {}

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

        // Animated clear color
        float t = float(frm.time_sec);
        VkClearValue clear{.color={{0.2f + 0.2f*std::sinf(t*0.7f), 0.15f + 0.15f*std::sinf(t*1.1f), 0.18f + 0.18f*std::cosf(t*0.9f), 1.0f}}};
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color.imageView = target.view; color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; color.storeOp = VK_ATTACHMENT_STORE_OP_STORE; color.clearValue = clear;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO}; ri.renderArea = {{0,0}, frm.extent}; ri.layerCount = 1; ri.colorAttachmentCount = 1; ri.pColorAttachments = &color;
        vkCmdBeginRendering(cmd, &ri);
        // no draw calls, just clear
        vkCmdEndRendering(cmd);

        barrier(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    }
};

int main(){
    VulkanEngine engine;
    engine.configure_window(1280, 720, "HinaCloth Visualizer: Simple Clear");
    engine.set_renderer(std::make_unique<SimpleClearRenderer>());
    engine.init();
    engine.run();
    engine.cleanup();
    return 0;
}


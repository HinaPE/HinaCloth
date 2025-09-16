#include "imgui_layer.h"
#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_vulkan.h"

namespace {
inline void checkVkResult(VkResult result, const char* expression) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string("Vulkan error (") + std::to_string(static_cast<int>(result)) + ") at " + expression);
    }
}
} // namespace

#ifndef VK_CHECK
#define VK_CHECK(expr) checkVkResult((expr), #expr)
#endif

bool ImGuiLayer::init(SDL_Window* window,
                      VkInstance instance,
                      VkPhysicalDevice physicalDevice,
                      VkDevice device,
                      VkQueue graphicsQueue,
                      uint32_t graphicsQueueFamily,
                      VkFormat swapchainFormat,
                      uint32_t swapchainImageCount)
{
    std::array<VkDescriptorPoolSize, 11> pool_sizes{{
        {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000},
    }};
    VkDescriptorPoolCreateInfo dpci{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
        .pNext = nullptr,
        .flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT,
        .maxSets = 1000u * (uint32_t)pool_sizes.size(),
        .poolSizeCount = (uint32_t)pool_sizes.size(),
        .pPoolSizes = pool_sizes.data()
    };
    VK_CHECK(vkCreateDescriptorPool(device, &dpci, nullptr, &pool_));

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // 开启 Docking + 多视口
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::StyleColorsDark();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiStyle& st = ImGui::GetStyle();
        st.WindowRounding = 0.0f;
        st.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    if (!ImGui_ImplSDL3_InitForVulkan(window)) {
        ImGui::DestroyContext();
        if (pool_) {
            vkDestroyDescriptorPool(device, pool_, nullptr);
            pool_ = VK_NULL_HANDLE;
        }
        return false;
    }

    ImGui_ImplVulkan_InitInfo ii{};
    ii.ApiVersion = VK_API_VERSION_1_3;
    ii.Instance = instance;
    ii.PhysicalDevice = physicalDevice;
    ii.Device = device;
    ii.QueueFamily = graphicsQueueFamily;
    ii.Queue = graphicsQueue;
    ii.DescriptorPool = pool_;
    ii.MinImageCount = swapchainImageCount;
    ii.ImageCount = swapchainImageCount;
    ii.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    ii.Allocator = nullptr;
    ii.CheckVkResultFn = [](VkResult e){ VK_CHECK(e); };
    ii.UseDynamicRendering = true;

    VkPipelineRenderingCreateInfo prci{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
        .pNext = nullptr,
        .viewMask = 0,
        .colorAttachmentCount = 1,
        .pColorAttachmentFormats = &swapchainFormat,
        .depthAttachmentFormat = VK_FORMAT_UNDEFINED,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED
    };
    ii.PipelineRenderingCreateInfo = prci;

    if (!ImGui_ImplVulkan_Init(&ii)) return false;

    colorFormat_ = swapchainFormat;
    inited_ = true;
    return true;
}

void ImGuiLayer::shutdown(VkDevice device)
{
    if (!inited_) return;
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    if (pool_) { vkDestroyDescriptorPool(device, pool_, nullptr); pool_ = VK_NULL_HANDLE; }
    inited_ = false;
}

void ImGuiLayer::process_event(const SDL_Event* e)
{
    if (!inited_ || !e) return;
    ImGui_ImplSDL3_ProcessEvent(e);
}

void ImGuiLayer::new_frame()
{
    if (!inited_) return;
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    for (auto& fn : panels_) fn();
}

void ImGuiLayer::render_overlay(VkCommandBuffer cmd,
                                VkImage swapchainImage,
                                VkImageView swapchainView,
                                VkExtent2D extent,
                                VkImageLayout previousLayout)
{
    if (!inited_) return;

    VkImageMemoryBarrier2 toColor{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
        .srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        .dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT,
        .oldLayout = previousLayout,
        .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .image = swapchainImage,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1}
    };
    VkDependencyInfo dep1{
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pNext = nullptr,
        .dependencyFlags = 0,
        .memoryBarrierCount = 0,
        .pMemoryBarriers = nullptr,
        .bufferMemoryBarrierCount = 0,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 1,
        .pImageMemoryBarriers = &toColor
    };
    vkCmdPipelineBarrier2(cmd, &dep1);

    VkRenderingAttachmentInfo color{
        .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
        .pNext = nullptr,
        .imageView = swapchainView,
        .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .resolveMode = VK_RESOLVE_MODE_NONE,
        .resolveImageView = VK_NULL_HANDLE,
        .resolveImageLayout = VK_IMAGE_LAYOUT_UNDEFINED,
        .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
        .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
        .clearValue = {}
    };
    VkRenderingInfo ri{
        .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
        .pNext = nullptr,
        .flags = 0,
        .renderArea = {{0,0}, extent},
        .layerCount = 1,
        .viewMask = 0,
        .colorAttachmentCount = 1,
        .pColorAttachments = &color,
        .pDepthAttachment = nullptr,
        .pStencilAttachment = nullptr
    };
    vkCmdBeginRendering(cmd, &ri);

    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);

    vkCmdEndRendering(cmd);

    // 多视口（平台窗口）
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }

    VkImageMemoryBarrier2 toPresent{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        .srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_NONE,
        .dstAccessMask = 0,
        .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        .image = swapchainImage,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1}
    };
    VkDependencyInfo dep2{
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pNext = nullptr,
        .dependencyFlags = 0,
        .memoryBarrierCount = 0,
        .pMemoryBarriers = nullptr,
        .bufferMemoryBarrierCount = 0,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 1,
        .pImageMemoryBarriers = &toPresent
    };
    vkCmdPipelineBarrier2(cmd, &dep2);
}

void ImGuiLayer::set_min_image_count(uint32_t count)
{
    if (!inited_) return;
    ImGui_ImplVulkan_SetMinImageCount(count);
}

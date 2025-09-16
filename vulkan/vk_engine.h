#ifndef HINACLOTH_VK_ENGINE_H
#define HINACLOTH_VK_ENGINE_H

#include "imgui_layer.h"
#include "renderer_iface.h"
#include "vk_mem_alloc.h"
#include <SDL3/SDL_vulkan.h>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>

struct DescriptorAllocator {
    struct PoolSizeRatio {
        VkDescriptorType type;
        float ratio;
    };
    VkDescriptorPool pool{};
    void init_pool(VkDevice device, uint32_t maxSets, std::span<PoolSizeRatio> ratios);
    void clear_descriptors(VkDevice device) const;
    void destroy_pool(VkDevice device) const;
    VkDescriptorSet allocate(VkDevice device, VkDescriptorSetLayout layout) const;
};

constexpr unsigned int FRAME_OVERLAP = 2;

class VulkanEngine {
public:
    void init();
    void run();
    void cleanup();
    void set_renderer(std::unique_ptr<IRenderer> r) { renderer_ = std::move(r); }
    VulkanEngine() = default;
    ~VulkanEngine() = default;
    VulkanEngine(const VulkanEngine&) = delete;
    VulkanEngine& operator=(const VulkanEngine&) = delete;
    VulkanEngine(VulkanEngine&&) noexcept = default;
    VulkanEngine& operator=(VulkanEngine&&) noexcept = default;

    struct {
        std::string name = "Vulkan Engine";
        int width = 1700;
        int height = 800;
        bool initialized{false};
        bool running{false};
        bool should_rendering{false};
        bool resize_requested{false};
        bool focused{true};
        bool minimized{false};
        uint64_t frame_number{0};
        double time_sec{0.0};
        double dt_sec{0.0};
    } state_;

private:
    void create_context(int window_width, int window_height, const char* app_name);
    void destroy_context();

    struct DeviceContext {
        VkInstance instance{};
        VkDebugUtilsMessengerEXT debug_messenger{};
        SDL_Window* window{nullptr};
        VkSurfaceKHR surface{};
        VkPhysicalDevice physical{};
        VkDevice device{};
        VkQueue graphics_queue{};
        VkQueue compute_queue{};
        VkQueue transfer_queue{};
        VkQueue present_queue{};
        uint32_t graphics_queue_family{};
        uint32_t compute_queue_family{};
        uint32_t transfer_queue_family{};
        uint32_t present_queue_family{};
        VmaAllocator allocator{};
        DescriptorAllocator descriptor_allocator;
    } ctx_;

    void create_swapchain(uint32_t width, uint32_t height);
    void destroy_swapchain();
    void recreate_swapchain();
    void create_offscreen_drawable(uint32_t width, uint32_t height);
    void destroy_offscreen_drawable();

    struct AllocatedImage {
        VkImage image{};
        VkImageView imageView{};
        VmaAllocation allocation{};
        VkExtent3D imageExtent{};
        VkFormat imageFormat{};
    };

    struct SwapchainSystem {
        VkSwapchainKHR swapchain{};
        VkFormat swapchain_image_format{};
        VkExtent2D swapchain_extent{};
        std::vector<VkImage> swapchain_images;
        std::vector<VkImageView> swapchain_image_views;
        AllocatedImage drawable_image;
        AllocatedImage depth_image{};
    } swapchain_;

    void create_command_buffers();
    void destroy_command_buffers();
    void begin_frame(uint32_t& imageIndex, VkCommandBuffer& cmd);
    void end_frame(uint32_t imageIndex, VkCommandBuffer cmd);

    struct FrameData {
        VkCommandPool commandPool{};
        VkCommandBuffer mainCommandBuffer{};
        VkSemaphore imageAcquired{};
        VkSemaphore renderComplete{};
        uint64_t submitted_timeline_value{0};
        std::vector<std::move_only_function<void()>> dq;
    } frames_[FRAME_OVERLAP];

    VkSemaphore render_timeline_{};
    uint64_t timeline_value_{0};

    void create_renderer();
    void destroy_renderer();
    std::unique_ptr<IRenderer> renderer_;
    RendererCaps renderer_caps_{};

    void create_imgui();
    void destroy_imgui();
    std::unique_ptr<ImGuiLayer> ui_;
    std::vector<std::move_only_function<void()>> mdq_;
};

#endif // HINACLOTH_VK_ENGINE_H

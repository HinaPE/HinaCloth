#ifndef HINACLOTH_VK_ENGINE_H
#define HINACLOTH_VK_ENGINE_H

#include "vk_mem_alloc.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <stdexcept>
#include <vector>

// ============================================================================
// Utility Macros
// ============================================================================
#ifndef VK_CHECK
#define VK_CHECK(x) do { VkResult _vk_check_res = (x); if (_vk_check_res != VK_SUCCESS) { throw std::runtime_error(std::string("Vulkan error ") + std::to_string(_vk_check_res) + " at " #x); } } while (false)
#endif
#ifndef IF_NOT_NULL_DO
#define IF_NOT_NULL_DO(ptr, stmt) do { if ((ptr) != nullptr) { stmt; } } while (false)
#endif
#ifndef IF_NOT_NULL_DO_AND_SET
#define IF_NOT_NULL_DO_AND_SET(ptr, stmt, val) do { if ((ptr) != nullptr) { stmt; (ptr) = (val); } } while (false)
#endif
#ifndef REQUIRE_TRUE
#define REQUIRE_TRUE(expr, msg) do { if (!(expr)) { throw std::runtime_error(std::string("Check failed: ") + #expr + " | " + (msg)); } } while (false)
#endif

namespace HinaPE {
class ClothData;
} // namespace HinaPE

struct DescriptorAllocator {
    struct PoolSizeRatio {
        VkDescriptorType type;
        float ratio;
    };
    VkDescriptorPool pool{};
    void init_pool(VkDevice device, uint32_t maxSets, std::span<const PoolSizeRatio> ratios);
    void clear_descriptors(VkDevice device) const;
    void destroy_pool(VkDevice device) const;
    VkDescriptorSet allocate(VkDevice device, VkDescriptorSetLayout layout) const;
};

inline constexpr unsigned int FRAME_OVERLAP = 2;

// ============================================================================
// Renderer Interface Contracts
// ============================================================================
struct EngineContext {
    VkInstance instance{};
    VkPhysicalDevice physical{};
    VkDevice device{};
    VmaAllocator allocator{};
    DescriptorAllocator* descriptorAllocator{};
    SDL_Window* window{};
    VkQueue graphics_queue{};
    VkQueue compute_queue{};
    VkQueue transfer_queue{};
    VkQueue present_queue{};
    uint32_t graphics_queue_family{};
    uint32_t compute_queue_family{};
    uint32_t transfer_queue_family{};
    uint32_t present_queue_family{};
};

struct FrameContext {
    uint64_t frame_index{};
    uint32_t image_index{};
    VkExtent2D extent{};
    VkFormat swapchain_format{};
    double dt_sec{};
    double time_sec{};
    VkImage swapchain_image{};
    VkImageView swapchain_image_view{};
    VkImage offscreen_image{};
    VkImageView offscreen_image_view{};
    VkImage depth_image{VK_NULL_HANDLE};
    VkImageView depth_image_view{VK_NULL_HANDLE};
};

struct RendererCaps {
    uint32_t api_version{};
    uint32_t frames_in_flight{FRAME_OVERLAP};
    VkBool32 dynamic_rendering{VK_TRUE};
    VkBool32 timeline_semaphore{VK_TRUE};
    VkBool32 descriptor_indexing{VK_TRUE};
    VkBool32 buffer_device_address{VK_TRUE};
    VkBool32 uses_depth{VK_FALSE};
    VkBool32 uses_offscreen{VK_TRUE};
};

struct RendererStats {
    uint64_t draw_calls{};
    uint64_t dispatches{};
    uint64_t triangles{};
    double cpu_ms{};
    double gpu_ms{};
};

class IRenderer {
public:
    virtual ~IRenderer() = default;
    virtual void initialize(const EngineContext& eng) = 0;
    virtual void destroy(const EngineContext& eng) = 0;
    virtual void on_swapchain_ready(const EngineContext& eng, const FrameContext& frm) { (void)eng; (void)frm; }
    virtual void on_swapchain_destroy(const EngineContext& eng) { (void)eng; }
    virtual void update(const EngineContext& eng, const FrameContext& frm) { (void)eng; (void)frm; }
    virtual void record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) = 0;
    virtual void record_compute(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) { (void)cmd; (void)eng; (void)frm; }
    virtual void on_event(const SDL_Event& e, const EngineContext& eng, const FrameContext* frm) { (void)e; (void)eng; (void)frm; }
    virtual void on_imgui(const EngineContext& eng, const FrameContext& frm) { (void)eng; (void)frm; }
    virtual void reload_assets(const EngineContext& eng) { (void)eng; }
    virtual void request_screenshot(const char* path) { (void)path; }
    virtual void get_capabilities(RendererCaps& out_caps) const { out_caps = RendererCaps{}; }
    [[nodiscard]] virtual RendererStats get_stats() const { return RendererStats{}; }
    virtual void set_option_int(const char* key, int v) { (void)key; (void)v; }
    virtual void set_option_float(const char* key, float v) { (void)key; (void)v; }
    virtual void set_option_str(const char* key, const char* v) { (void)key; (void)v; }
    virtual bool get_option_int(const char* key, int& v) const { (void)key; (void)v; return false; }
    virtual bool get_option_float(const char* key, float& v) const { (void)key; (void)v; return false; }
    virtual bool get_option_str(const char* key, const char*& v) const { (void)key; (void)v; return false; }
};

// ============================================================================
// Vulkan Engine
// ============================================================================
class VulkanEngine {
public:
    VulkanEngine();
    ~VulkanEngine();
    VulkanEngine(const VulkanEngine&) = delete;
    VulkanEngine& operator=(const VulkanEngine&) = delete;
    VulkanEngine(VulkanEngine&&) noexcept = default;
    VulkanEngine& operator=(VulkanEngine&&) noexcept = default;

    void init();
    void run();
    void cleanup();

    void set_renderer(std::unique_ptr<IRenderer> r) { renderer_ = std::move(r); }

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
    // -- context --
    void create_context(int window_width, int window_height, const char* app_name);
    void destroy_context();
    EngineContext make_engine_context() const;
    FrameContext make_frame_context(uint64_t frame_index, uint32_t image_index, VkExtent2D extent) const;
    void blit_offscreen_to_swapchain(VkCommandBuffer cmd, uint32_t imageIndex, VkExtent2D extent);

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
    } ctx_{};

    // -- swapchain --
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
    } swapchain_{};

    // -- command submission --
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
    } frames_[FRAME_OVERLAP]{};

    VkSemaphore render_timeline_{};
    uint64_t timeline_value_{0};

    // -- renderer integration --
    void create_renderer();
    void destroy_renderer();
    std::unique_ptr<IRenderer> renderer_;
    RendererCaps renderer_caps_{};

    // -- Dear ImGui integration --
    struct UiSystem;
    void create_imgui();
    void destroy_imgui();
    std::unique_ptr<UiSystem> ui_;
    std::vector<std::move_only_function<void()>> mdq_;
};

#endif // HINACLOTH_VK_ENGINE_H


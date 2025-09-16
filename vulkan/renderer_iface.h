#ifndef HINACLOTH_RENDERER_IFACE_H
#define HINACLOTH_RENDERER_IFACE_H

#include "vk_mem_alloc.h"
#include <SDL3/SDL.h>
#include <stdexcept>
#include <string>
#include <vulkan/vulkan.h>

// clang-format off
#ifndef VK_CHECK
#define VK_CHECK(x) do{ VkResult _e=(x); if(_e!=VK_SUCCESS){ throw std::runtime_error("Vulkan error "+std::to_string(_e)); } }while(0)
#endif
#ifndef IF_NOT_NULL_DO
#define IF_NOT_NULL_DO(ptr, stmt) do{ if((ptr)!=nullptr){ stmt; } }while(0)
#endif
#ifndef IF_NOT_NULL_DO_AND_SET
#define IF_NOT_NULL_DO_AND_SET(ptr, stmt, val) do{ if((ptr)!=nullptr){ stmt; (ptr)=val; } }while(0)
#endif
#ifndef REQUIRE_TRUE
#define REQUIRE_TRUE(expr,msg) do{ if(!(expr)){ throw std::runtime_error(std::string("Check failed: ")+#expr+" | "+(msg)); } }while(0)
#endif

struct DescriptorAllocator;

struct EngineContext
{
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

struct FrameContext
{
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

struct RendererCaps
{
    uint32_t api_version{};
    uint32_t frames_in_flight{2};
    VkBool32 dynamic_rendering{VK_TRUE};
    VkBool32 timeline_semaphore{VK_TRUE};
    VkBool32 descriptor_indexing{VK_TRUE};
    VkBool32 buffer_device_address{VK_TRUE};
    VkBool32 uses_depth{VK_FALSE};
    VkBool32 uses_offscreen{VK_TRUE};
};

struct RendererStats
{
    uint64_t draw_calls{};
    uint64_t dispatches{};
    uint64_t triangles{};
    double cpu_ms{};
    double gpu_ms{};
};

class IRenderer
{
public:
    virtual ~IRenderer() = default;

    virtual void initialize(const EngineContext& eng) = 0;
    virtual void destroy(const EngineContext& eng) = 0;

    virtual void on_swapchain_ready(const EngineContext& eng, const FrameContext& frm) { (void) eng; (void) frm; }
    virtual void on_swapchain_destroy(const EngineContext& eng) { (void) eng; }

    virtual void update(const EngineContext& eng, const FrameContext& frm) { (void) eng; (void) frm; }
    virtual void record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) = 0;
    virtual void record_compute(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) { (void) cmd; (void) eng; (void) frm; }

    virtual void on_event(const SDL_Event& e, const EngineContext& eng, const FrameContext* frm) { (void) e; (void) eng; (void) frm; }
    virtual void on_imgui(const EngineContext& eng, const FrameContext& frm) { (void) eng; (void) frm; }

    virtual void reload_assets(const EngineContext& eng) { (void) eng; }
    virtual void request_screenshot(const char* path) { (void) path; }

    virtual void get_capabilities(RendererCaps& out_caps) const { out_caps = RendererCaps{}; }
    [[nodiscard]] virtual RendererStats get_stats() const { return RendererStats{}; }

    virtual void set_option_int(const char* key, int v) { (void) key; (void) v; }
    virtual void set_option_float(const char* key, float v) { (void) key; (void) v; }
    virtual void set_option_str(const char* key, const char* v) { (void) key; (void) v; }
    virtual bool get_option_int(const char* key, int& v) const { (void) key; (void) v; return false; }
    virtual bool get_option_float(const char* key, float& v) const { (void) key; (void) v; return false; }
    virtual bool get_option_str(const char* key, const char*& v) const { (void) key; (void) v; return false; }
};
// clang-format on
#endif // HINACLOTH_RENDERER_IFACE_H

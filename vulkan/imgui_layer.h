#ifndef HINACLOTH_IMGUI_LAYER_H
#define HINACLOTH_IMGUI_LAYER_H

#include <vulkan/vulkan.h>
#include <SDL3/SDL.h>
#include <functional>
#include <vector>

class ImGuiLayer
{
public:
    bool init(SDL_Window* window,
              VkInstance instance,
              VkPhysicalDevice physicalDevice,
              VkDevice device,
              VkQueue graphicsQueue,
              uint32_t graphicsQueueFamily,
              VkFormat swapchainFormat,
              uint32_t swapchainImageCount);

    void shutdown(VkDevice device);
    void process_event(const SDL_Event* e);
    void new_frame();

    void render_overlay(VkCommandBuffer cmd,
                        VkImage swapchainImage,
                        VkImageView swapchainView,
                        VkExtent2D extent,
                        VkImageLayout previousLayout);

    using PanelFn = std::function<void()>;
    void add_panel(PanelFn fn) { panels_.push_back(std::move(fn)); }

    void set_min_image_count(uint32_t count);

private:
    VkDescriptorPool pool_{};
    bool inited_{false};
    VkFormat colorFormat_{};
    std::vector<PanelFn> panels_;
};

#endif // HINACLOTH_IMGUI_LAYER_H

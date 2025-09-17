#include "vulkan/vk_engine.h"
#include "vulkan/cloth_renderer.h"

std::unique_ptr<IRenderer> CreateDefaultComputeRenderer()
{
    return CreateClothRenderer();
}

int main()
{
    VulkanEngine engine;
    engine.init();
    engine.run();
    engine.cleanup();
    return 0;
}

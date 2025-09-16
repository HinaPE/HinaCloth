#include "vulkan/vk_engine.h"

std::unique_ptr<IRenderer> CreateDefaultComputeRenderer()
{
    return nullptr;
}

int main()
{
    VulkanEngine engine;
    engine.init();
    engine.run();
    engine.cleanup();
    return 0;
}
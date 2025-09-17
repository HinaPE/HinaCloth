#include "cloth_renderer.h"

#include "renderer_iface.h"
#include "vk_engine.h"

#include "aligned_allocator.h"
#include "cloth_data.h"
#include "xpbd.h"

#include <SDL3/SDL.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>

#include "shaders/cloth_vert_spv.inc"
#include "shaders/cloth_frag_spv.inc"

namespace {

constexpr uint32_t kGridWidth  = 40;
constexpr uint32_t kGridHeight = 40;
constexpr float    kGridSpacing = 0.025f;

constexpr float kCameraPitchMin = -1.4f;
constexpr float kCameraPitchMax = -0.05f;
constexpr float kCameraDistanceMin = 0.3f;
constexpr float kCameraDistanceMax = 15.0f;

VkShaderModule create_shader_module(VkDevice device, std::span<const uint32_t> code) {
    VkShaderModuleCreateInfo ci{.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .codeSize = code.size() * sizeof(uint32_t),
        .pCode = code.data()};
    VkShaderModule module = VK_NULL_HANDLE;
    VK_CHECK(vkCreateShaderModule(device, &ci, nullptr, &module));
    return module;
}

} // namespace

std::unique_ptr<IRenderer> CreateClothRenderer() {
    return std::make_unique<ClothRenderer>();
}

ClothRenderer::ClothRenderer() : cloth_(64) {
    params_.time_step = 1.0f / 60.0f;
    params_.substeps = 4;
    params_.solver_iterations = 8;
    params_.enable_distance_constraints = true;
    params_.enable_bending_constraints = false;
    params_.velocity_damping = 0.005f;
}

ClothRenderer::~ClothRenderer() = default;

void ClothRenderer::initialize(const EngineContext& eng) {
    eng_ = eng;
    blueprint_ = make_grid_blueprint(kGridWidth, kGridHeight, kGridSpacing);
    load_cloth(cloth_, blueprint_);

    const float width_world  = static_cast<float>(blueprint_.width - 1) * blueprint_.spacing;
    const float height_world = static_cast<float>(blueprint_.height - 1) * blueprint_.spacing;
    target_ = Vec3{0.0f, 0.0f, 0.0f};
    camera_distance_ = std::max(width_world, height_world) * 1.8f;
    camera_distance_ = std::clamp(camera_distance_, kCameraDistanceMin, kCameraDistanceMax);

    ensure_buffers(eng);
    pending_upload_ = true;
    initialized_    = true;
}

void ClothRenderer::destroy(const EngineContext& eng) {
    destroy_pipeline(eng);
    destroy_buffers(eng);
    initialized_ = false;
}

void ClothRenderer::on_swapchain_ready(const EngineContext& eng, const FrameContext& frm) {
    swapchain_format_ = frm.swapchain_format;
    create_pipeline(eng, frm.swapchain_format);
}

void ClothRenderer::on_swapchain_destroy(const EngineContext& eng) {
    destroy_pipeline(eng);
}

void ClothRenderer::update(const EngineContext&, const FrameContext& frm) {
    if (!initialized_) return;

    const double step = static_cast<double>(params_.time_step);
    accumulator_ += frm.dt_sec;
    int iterations = 0;
    while (simulate_ && accumulator_ >= step) {
        xpbd_step_native(cloth_, params_);
        accumulator_ -= step;
        ++iterations;
    }

    const bool needs_upload = pending_upload_ || (simulate_ && iterations > 0);
    if (!needs_upload) {
        stats_.cpu_ms = frm.dt_sec * 1000.0;
        return;
    }

    build_vertices();
    upload_frame_buffers(static_cast<uint32_t>(frm.frame_index % FRAME_OVERLAP));
    pending_upload_ = false;

    stats_.draw_calls = 0;
    stats_.triangles = 0;
    stats_.dispatches = 0;
    stats_.cpu_ms = frm.dt_sec * 1000.0;
    stats_.gpu_ms = 0.0;
}

void ClothRenderer::record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) {
    stats_.draw_calls = 0;
    stats_.triangles = 0;
    stats_.dispatches = 0;
    stats_.gpu_ms = 0.0;
    if (!pipelines_ready_) {
        return;
    }

    VkViewport viewport{
        .x = 0.0f,
        .y = 0.0f,
        .width = static_cast<float>(frm.extent.width),
        .height = static_cast<float>(frm.extent.height),
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
    };
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{
        .offset = {0, 0},
        .extent = frm.extent,
    };
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    VkImageMemoryBarrier2 off_to_color{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = offscreen_ready_ ? VK_PIPELINE_STAGE_2_TRANSFER_BIT : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
        .srcAccessMask = offscreen_ready_ ? VK_ACCESS_2_TRANSFER_READ_BIT : 0u,
        .dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        .dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
        .oldLayout = offscreen_ready_ ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
        .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .image = frm.offscreen_image,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 1u, 0u, 1u},
    };
    VkDependencyInfo dep_off{
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pNext = nullptr,
        .dependencyFlags = 0u,
        .memoryBarrierCount = 0u,
        .pMemoryBarriers = nullptr,
        .bufferMemoryBarrierCount = 0u,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 1u,
        .pImageMemoryBarriers = &off_to_color,
    };
    vkCmdPipelineBarrier2(cmd, &dep_off);

    VkClearValue clear_color{};
    clear_color.color.float32[0] = 0.04f;
    clear_color.color.float32[1] = 0.05f;
    clear_color.color.float32[2] = 0.08f;
    clear_color.color.float32[3] = 1.0f;

    VkRenderingAttachmentInfo color_attachment{
        .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
        .pNext = nullptr,
        .imageView = frm.offscreen_image_view,
        .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .resolveMode = VK_RESOLVE_MODE_NONE,
        .resolveImageView = VK_NULL_HANDLE,
        .resolveImageLayout = VK_IMAGE_LAYOUT_UNDEFINED,
        .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
        .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
        .clearValue = clear_color,
    };

    VkRenderingInfo rendering_info{
        .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .renderArea = {{0, 0}, frm.extent},
        .layerCount = 1u,
        .viewMask = 0u,
        .colorAttachmentCount = 1u,
        .pColorAttachments = &color_attachment,
        .pDepthAttachment = nullptr,
        .pStencilAttachment = nullptr,
    };

    vkCmdBeginRendering(cmd, &rendering_info);

    Vec3 offset{
        camera_distance_ * std::cos(camera_pitch_) * std::cos(camera_yaw_),
        camera_distance_ * std::sin(camera_pitch_),
        camera_distance_ * std::cos(camera_pitch_) * std::sin(camera_yaw_),
    };
    Vec3 eye{target_.x + offset.x, target_.y + offset.y, target_.z + offset.z};
    std::array<float, 16> view = make_look_at(eye, target_, Vec3{0.0f, 1.0f, 0.0f});
    const float aspect = static_cast<float>(frm.extent.width) / std::max(1.0f, static_cast<float>(frm.extent.height));
    std::array<float, 16> proj = make_perspective(std::numbers::pi_v<float> / 4.0f, aspect, 0.05f, 100.0f);
    std::array<float, 16> mvp = multiply(proj, view);

    PushConstants push{};
    push.mvp = mvp;
    push.point_size = point_size_pixels_;
    push.line_width = line_width_pixels_;

    const uint32_t frame_slot = static_cast<uint32_t>(frm.frame_index % FRAME_OVERLAP);
    VkDeviceSize vb_offset = 0;

    if (draw_constraints_ && line_vertex_count_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_);
        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &push);
        vkCmdBindVertexBuffers(cmd, 0, 1, &line_buffers_[frame_slot].buffer, &vb_offset);
        vkCmdSetLineWidth(cmd, line_width_pixels_);
        vkCmdDraw(cmd, static_cast<uint32_t>(line_vertex_count_), 1u, 0u, 0u);
        stats_.draw_calls++;
        stats_.triangles += line_vertex_count_ / 2;
    }

    if (draw_vertices_ && point_count_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, point_pipeline_);
        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &push);
        vkCmdBindVertexBuffers(cmd, 0, 1, &point_buffers_[frame_slot].buffer, &vb_offset);
        vkCmdDraw(cmd, static_cast<uint32_t>(point_count_), 1u, 0u, 0u);
        stats_.draw_calls++;
    }

    vkCmdEndRendering(cmd);

    VkImageMemoryBarrier2 off_to_src{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        .srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT,
        .dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        .image = frm.offscreen_image,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 1u, 0u, 1u},
    };
    VkImageMemoryBarrier2 swap_to_dst{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
        .srcAccessMask = 0u,
        .dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT,
        .dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
        .newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .image = frm.swapchain_image,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 1u, 0u, 1u},
    };
    VkImageMemoryBarrier2 transfer_barriers[2]{off_to_src, swap_to_dst};
    VkDependencyInfo dep_transfer{
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pNext = nullptr,
        .dependencyFlags = 0u,
        .memoryBarrierCount = 0u,
        .pMemoryBarriers = nullptr,
        .bufferMemoryBarrierCount = 0u,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 2u,
        .pImageMemoryBarriers = transfer_barriers,
    };
    vkCmdPipelineBarrier2(cmd, &dep_transfer);

    VkImageCopy blit_region{
        .srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 0u, 1u},
        .srcOffset = {0, 0, 0},
        .dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 0u, 1u},
        .dstOffset = {0, 0, 0},
        .extent = {frm.extent.width, frm.extent.height, 1u},
    };
    vkCmdCopyImage(
        cmd,
        frm.offscreen_image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        frm.swapchain_image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1u,
        &blit_region);

    VkImageMemoryBarrier2 swap_visible{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
        .pNext = nullptr,
        .srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT,
        .srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
        .dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .image = frm.swapchain_image,
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 1u, 0u, 1u},
    };
    VkDependencyInfo dep_visible{
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pNext = nullptr,
        .dependencyFlags = 0u,
        .memoryBarrierCount = 0u,
        .pMemoryBarriers = nullptr,
        .bufferMemoryBarrierCount = 0u,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 1u,
        .pImageMemoryBarriers = &swap_visible,
    };
    vkCmdPipelineBarrier2(cmd, &dep_visible);

    offscreen_ready_ = true;
}


void ClothRenderer::on_event(const SDL_Event& e, const EngineContext&, const FrameContext*) {
    switch (e.type) {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (e.button.button == SDL_BUTTON_RIGHT) {
            rotating_ = true;
            last_mouse_x_ = e.button.x;
            last_mouse_y_ = e.button.y;
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_RIGHT) {
            rotating_ = false;
        }
        break;
    case SDL_EVENT_MOUSE_MOTION:
        if (rotating_) {
            const float dx = static_cast<float>(e.motion.x - last_mouse_x_) * 0.005f;
            const float dy = static_cast<float>(e.motion.y - last_mouse_y_) * 0.005f;
            camera_yaw_ += dx;
            camera_pitch_ = std::clamp(camera_pitch_ - dy, kCameraPitchMin, kCameraPitchMax);
            last_mouse_x_ = e.motion.x;
            last_mouse_y_ = e.motion.y;
        }
        break;
    case SDL_EVENT_MOUSE_WHEEL: {
        const float delta = static_cast<float>(e.wheel.y) * 0.2f;
        camera_distance_ = std::clamp(camera_distance_ * (1.0f - delta * 0.1f), kCameraDistanceMin, kCameraDistanceMax);
        break;
    }
    default: break;
    }
}

void ClothRenderer::on_imgui(const EngineContext&, const FrameContext&) {
    if (!ImGui::Begin("Cloth Renderer")) {
        ImGui::End();
        return;
    }

    ImGui::Text("Particles: %zu", cloth_.num_particles());
    ImGui::Text("Constraints: %zu", cloth_.num_edges());
    ImGui::Checkbox("Simulate", &simulate_);
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        reset_simulation();
        pending_upload_ = true;
    }
    ImGui::Checkbox("Show Vertices", &draw_vertices_);
    ImGui::Checkbox("Show Constraints", &draw_constraints_);
    ImGui::SliderFloat("Point Size", &point_size_pixels_, 1.0f, 20.0f);
    ImGui::SliderFloat("Line Width", &line_width_pixels_, 0.5f, 5.0f);

    ImGui::Separator();
    ImGui::Text("Camera distance: %.2f", camera_distance_);
    ImGui::Text("Yaw: %.2f Pitch: %.2f", camera_yaw_, camera_pitch_);

    ImGui::Separator();
    ImGui::Text("Draw calls: %llu", static_cast<unsigned long long>(stats_.draw_calls));
    ImGui::Text("Edges rendered: %zu", line_vertex_count_ / 2);

    ImGui::End();
}

void ClothRenderer::reload_assets(const EngineContext& eng) {
    destroy_pipeline(eng);
    offscreen_ready_ = false;
    if (swapchain_format_ != VK_FORMAT_UNDEFINED) {
        create_pipeline(eng, swapchain_format_);
    }
}

void ClothRenderer::get_capabilities(RendererCaps& out_caps) const {
    out_caps = RendererCaps{};
    out_caps.uses_depth = VK_FALSE;
    out_caps.uses_offscreen = VK_FALSE;
    out_caps.dynamic_rendering = VK_TRUE;
}

RendererStats ClothRenderer::get_stats() const {
    return stats_;
}

void ClothRenderer::destroy_buffers(const EngineContext& eng) {
    for (auto& buf : point_buffers_) {
        if (buf.buffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(eng.allocator, buf.buffer, buf.allocation);
            buf = {};
        }
    }
    for (auto& buf : line_buffers_) {
        if (buf.buffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(eng.allocator, buf.buffer, buf.allocation);
            buf = {};
        }
    }
}

void ClothRenderer::destroy_pipeline(const EngineContext& eng) {
    IF_NOT_NULL_DO_AND_SET(point_pipeline_, vkDestroyPipeline(eng.device, point_pipeline_, nullptr), VK_NULL_HANDLE);
    IF_NOT_NULL_DO_AND_SET(line_pipeline_, vkDestroyPipeline(eng.device, line_pipeline_, nullptr), VK_NULL_HANDLE);
    IF_NOT_NULL_DO_AND_SET(pipeline_layout_, vkDestroyPipelineLayout(eng.device, pipeline_layout_, nullptr), VK_NULL_HANDLE);
    IF_NOT_NULL_DO_AND_SET(vert_module_, vkDestroyShaderModule(eng.device, vert_module_, nullptr), VK_NULL_HANDLE);
    IF_NOT_NULL_DO_AND_SET(frag_module_, vkDestroyShaderModule(eng.device, frag_module_, nullptr), VK_NULL_HANDLE);
    pipelines_ready_ = false;
}

void ClothRenderer::create_pipeline(const EngineContext& eng, VkFormat swapchain_format) {
    destroy_pipeline(eng);

    vert_module_ = create_shader_module(eng.device, std::span<const uint32_t>(kClothVertSpv, kClothVertSpvCount));
    frag_module_ = create_shader_module(eng.device, std::span<const uint32_t>(kClothFragSpv, kClothFragSpvCount));

    VkPushConstantRange push_range{
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
        .offset = 0u,
        .size = sizeof(PushConstants),
    };

    VkPipelineLayoutCreateInfo plci{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .setLayoutCount = 0u,
        .pSetLayouts = nullptr,
        .pushConstantRangeCount = 1u,
        .pPushConstantRanges = &push_range,
    };
    VK_CHECK(vkCreatePipelineLayout(eng.device, &plci, nullptr, &pipeline_layout_));

    VkPipelineShaderStageCreateInfo stages[2]{
        VkPipelineShaderStageCreateInfo{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext = nullptr,
            .flags = 0u,
            .stage = VK_SHADER_STAGE_VERTEX_BIT,
            .module = vert_module_,
            .pName = "main",
            .pSpecializationInfo = nullptr,
        },
        VkPipelineShaderStageCreateInfo{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext = nullptr,
            .flags = 0u,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .module = frag_module_,
            .pName = "main",
            .pSpecializationInfo = nullptr,
        },
    };

    VkVertexInputBindingDescription binding{
        .binding = 0u,
        .stride = sizeof(Vertex),
        .inputRate = VK_VERTEX_INPUT_RATE_VERTEX,
    };
    std::array<VkVertexInputAttributeDescription, 2> attributes{
        VkVertexInputAttributeDescription{.location = 0u, .binding = 0u, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(Vertex, position)},
        VkVertexInputAttributeDescription{.location = 1u, .binding = 0u, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(Vertex, color)},
    };

    VkPipelineVertexInputStateCreateInfo visi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .vertexBindingDescriptionCount = 1u,
        .pVertexBindingDescriptions = &binding,
        .vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size()),
        .pVertexAttributeDescriptions = attributes.data(),
    };

    VkPipelineInputAssemblyStateCreateInfo iasi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST,
        .primitiveRestartEnable = VK_FALSE,
    };

    VkPipelineViewportStateCreateInfo pvsi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .viewportCount = 1u,
        .pViewports = nullptr,
        .scissorCount = 1u,
        .pScissors = nullptr,
    };

    VkPipelineRasterizationStateCreateInfo rsi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .depthClampEnable = VK_FALSE,
        .rasterizerDiscardEnable = VK_FALSE,
        .polygonMode = VK_POLYGON_MODE_FILL,
        .cullMode = VK_CULL_MODE_NONE,
        .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
        .depthBiasEnable = VK_FALSE,
        .lineWidth = 1.0f,
    };

    VkPipelineMultisampleStateCreateInfo msi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT,
        .sampleShadingEnable = VK_FALSE,
    };

    VkPipelineColorBlendAttachmentState cb_attach{
        .blendEnable = VK_FALSE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstColorBlendFactor = VK_BLEND_FACTOR_ZERO,
        .colorBlendOp = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,
        .alphaBlendOp = VK_BLEND_OP_ADD,
        .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
    };

    VkPipelineColorBlendStateCreateInfo cbsi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .logicOpEnable = VK_FALSE,
        .logicOp = VK_LOGIC_OP_COPY,
        .attachmentCount = 1u,
        .pAttachments = &cb_attach,
    };

    std::array<VkDynamicState, 3> dynamic_states{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_LINE_WIDTH};
    VkPipelineDynamicStateCreateInfo dsi{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0u,
        .dynamicStateCount = static_cast<uint32_t>(dynamic_states.size()),
        .pDynamicStates = dynamic_states.data(),
    };

    VkPipelineRenderingCreateInfo prci{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
        .pNext = nullptr,
        .viewMask = 0u,
        .colorAttachmentCount = 1u,
        .pColorAttachmentFormats = &swapchain_format,
        .depthAttachmentFormat = VK_FORMAT_UNDEFINED,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
    };

    VkGraphicsPipelineCreateInfo gpci{
        .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
        .pNext = &prci,
        .flags = 0u,
        .stageCount = 2u,
        .pStages = stages,
        .pVertexInputState = &visi,
        .pInputAssemblyState = &iasi,
        .pViewportState = &pvsi,
        .pRasterizationState = &rsi,
        .pMultisampleState = &msi,
        .pDepthStencilState = nullptr,
        .pColorBlendState = &cbsi,
        .pDynamicState = &dsi,
        .layout = pipeline_layout_,
        .renderPass = VK_NULL_HANDLE,
        .subpass = 0u,
        .basePipelineHandle = VK_NULL_HANDLE,
        .basePipelineIndex = -1,
    };

    VK_CHECK(vkCreateGraphicsPipelines(eng.device, VK_NULL_HANDLE, 1u, &gpci, nullptr, &line_pipeline_));

    iasi.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    dynamic_states = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_LINE_WIDTH};
    VK_CHECK(vkCreateGraphicsPipelines(eng.device, VK_NULL_HANDLE, 1u, &gpci, nullptr, &point_pipeline_));

    pipelines_ready_ = true;
}

void ClothRenderer::ensure_buffers(const EngineContext& eng) {
    if (point_buffers_[0].buffer != VK_NULL_HANDLE) return;

    const VkDeviceSize point_size = static_cast<VkDeviceSize>(blueprint_.width * blueprint_.height * sizeof(Vertex));
    const VkDeviceSize line_size = static_cast<VkDeviceSize>(blueprint_.edge_i.size() * 2ull * sizeof(Vertex));

    auto alloc_buffer = [&](HostBuffer& dst, VkDeviceSize size) {
        VkBufferCreateInfo bci{
            .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
            .pNext = nullptr,
            .flags = 0u,
            .size = size,
            .usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .queueFamilyIndexCount = 0u,
            .pQueueFamilyIndices = nullptr,
        };
        VmaAllocationCreateInfo aci{
            .flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT,
            .usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST,
            .requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
            .preferredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            .memoryTypeBits = 0u,
            .pool = VK_NULL_HANDLE,
            .pUserData = nullptr,
            .priority = 1.0f,
        };
        VmaAllocationInfo info{};
        VK_CHECK(vmaCreateBuffer(eng.allocator, &bci, &aci, &dst.buffer, &dst.allocation, &info));
        dst.mapped = info.pMappedData;
        dst.size = size;
    };

    for (auto& buf : point_buffers_) alloc_buffer(buf, point_size);
    for (auto& buf : line_buffers_) alloc_buffer(buf, line_size);
}

void ClothRenderer::upload_frame_buffers(uint32_t frame_index) {
    const auto copy_to_buffer = [&](HostBuffer& buf, const std::vector<Vertex>& src, size_t count) {
        const VkDeviceSize bytes = static_cast<VkDeviceSize>(count * sizeof(Vertex));
        if (bytes > buf.size) {
            throw std::runtime_error("ClothRenderer::upload_frame_buffers overflow");
        }
        if (bytes > 0 && buf.mapped) {
            std::memcpy(buf.mapped, src.data(), static_cast<size_t>(bytes));
            vmaFlushAllocation(eng_.allocator, buf.allocation, 0, bytes);
        }
    };

    copy_to_buffer(point_buffers_[frame_index], cpu_points_, point_count_);
    copy_to_buffer(line_buffers_[frame_index], cpu_lines_, line_vertex_count_);
}

void ClothRenderer::build_vertices() {
    const auto particles = cloth_.particles();
    point_count_ = particles.n;
    cpu_points_.resize(point_count_);

    const auto px = particles.px.span();
    const auto py = particles.py.span();
    const auto pz = particles.pz.span();
    const auto pinned = particles.pinned.span();

    for (size_t i = 0; i < point_count_; ++i) {
        const bool is_pinned = pinned.empty() ? false : (pinned[i] != 0);
        auto& v = cpu_points_[i];
        v.position[0] = px[i];
        v.position[1] = py[i];
        v.position[2] = pz[i];
        if (is_pinned) {
            v.color[0] = 1.0f;
            v.color[1] = 0.3f;
            v.color[2] = 0.3f;
        } else {
            v.color[0] = 0.2f;
            v.color[1] = 0.7f;
            v.color[2] = 1.0f;
        }
    }

    const auto dist = cloth_.distance();
    line_vertex_count_ = dist.m * 2;
    cpu_lines_.resize(line_vertex_count_);

    const auto idx_i = dist.i.span();
    const auto idx_j = dist.j.span();
    const auto colors = dist.color.span();

    const auto color_for = [&](std::uint8_t c) -> std::array<float, 3> {
        switch (c % 4) {
        case 0: return {0.9f, 0.6f, 0.2f};
        case 1: return {0.2f, 0.9f, 0.4f};
        case 2: return {0.2f, 0.6f, 0.9f};
        default: return {0.8f, 0.3f, 0.8f};
        }
    };

    for (size_t c = 0; c < dist.m; ++c) {
        const size_t dst = c * 2;
        const std::uint32_t i = idx_i[c];
        const std::uint32_t j = idx_j[c];
        const auto color = color_for(colors.empty() ? static_cast<std::uint8_t>(c & 3) : colors[c]);

        cpu_lines_[dst + 0].position[0] = px[i];
        cpu_lines_[dst + 0].position[1] = py[i];
        cpu_lines_[dst + 0].position[2] = pz[i];
        cpu_lines_[dst + 0].color[0] = color[0];
        cpu_lines_[dst + 0].color[1] = color[1];
        cpu_lines_[dst + 0].color[2] = color[2];

        cpu_lines_[dst + 1].position[0] = px[j];
        cpu_lines_[dst + 1].position[1] = py[j];
        cpu_lines_[dst + 1].position[2] = pz[j];
        cpu_lines_[dst + 1].color[0] = color[0];
        cpu_lines_[dst + 1].color[1] = color[1];
        cpu_lines_[dst + 1].color[2] = color[2];
    }
}

ClothRenderer::ClothBlueprint ClothRenderer::make_grid_blueprint(uint32_t width, uint32_t height, float spacing) {
    if (width < 2 || height < 2) {
        throw std::invalid_argument("Cloth grid dimensions must be >= 2");
    }
    if (!(spacing > 0.0f)) {
        throw std::invalid_argument("Cloth grid spacing must be positive");
    }

    ClothBlueprint bp{};
    bp.width = width;
    bp.height = height;
    bp.spacing = spacing;

    const size_t particle_count = static_cast<size_t>(width) * static_cast<size_t>(height);
    bp.px.resize(particle_count);
    bp.py.resize(particle_count);
    bp.pz.assign(particle_count, 0.0f);
    bp.vx.assign(particle_count, 0.0f);
    bp.vy.assign(particle_count, 0.0f);
    bp.vz.assign(particle_count, 0.0f);
    bp.inv_mass.assign(particle_count, 1.0f);
    bp.pinned.assign(particle_count, 0u);

    const float diag_offset = spacing * 0.5f;

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            bp.px[idx] = static_cast<float>(x) * spacing;
            bp.py[idx] = static_cast<float>(height - 1 - y) * spacing + ((y % 2u) ? diag_offset : 0.0f);
            min_x = std::min(min_x, bp.px[idx]);
            max_x = std::max(max_x, bp.px[idx]);
            min_y = std::min(min_y, bp.py[idx]);
            max_y = std::max(max_y, bp.py[idx]);
            if (y == 0u) {
                bp.pinned[idx] = 1u;
                bp.inv_mass[idx] = 0.0f;
            }
        }
    }

    if (!bp.px.empty()) {
        const float center_x = 0.5f * (min_x + max_x);
        const float center_y = 0.5f * (min_y + max_y);
        for (size_t idx = 0; idx < bp.px.size(); ++idx) {
            bp.px[idx] -= center_x;
            bp.py[idx] -= center_y;
        }
    }

    const size_t horizontal = static_cast<size_t>(width - 1) * height;
    const size_t vertical = static_cast<size_t>(height - 1) * width;
    const size_t edge_count = horizontal + vertical;
    bp.edge_i.reserve(edge_count);
    bp.edge_j.reserve(edge_count);
    bp.rest.reserve(edge_count);
    bp.compliance.reserve(edge_count);
    bp.lambda.reserve(edge_count);
    bp.alpha.reserve(edge_count);
    bp.color.reserve(edge_count);

    auto append_edge = [&](uint32_t i, uint32_t j, uint8_t color) {
        bp.edge_i.push_back(i);
        bp.edge_j.push_back(j);
        bp.rest.push_back(spacing);
        bp.compliance.push_back(0.0f);
        bp.lambda.push_back(0.0f);
        bp.alpha.push_back(0.0f);
        bp.color.push_back(color);
    };

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x + 1u < width; ++x) {
            const uint32_t idx = y * width + x;
            append_edge(idx, idx + 1u, static_cast<uint8_t>(x & 1u));
        }
    }
    for (uint32_t y = 0; y + 1u < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t idx = y * width + x;
            append_edge(idx, idx + width, static_cast<uint8_t>(2u + (y & 1u)));
        }
    }
    return bp;
}

void ClothRenderer::load_cloth(HinaPE::ClothData& cloth, const ClothBlueprint& bp) {
    cloth.allocate_particles(bp.px.size());
    cloth.allocate_distance(bp.edge_i.size());
    cloth.allocate_triangles(0);
    cloth.allocate_bending(0);
    cloth.allocate_tri_elastic(0);

    auto particles = cloth.particles();
    std::copy(bp.px.begin(), bp.px.end(), particles.px.span().begin());
    std::copy(bp.py.begin(), bp.py.end(), particles.py.span().begin());
    std::copy(bp.pz.begin(), bp.pz.end(), particles.pz.span().begin());
    std::copy(bp.vx.begin(), bp.vx.end(), particles.vx.span().begin());
    std::copy(bp.vy.begin(), bp.vy.end(), particles.vy.span().begin());
    std::copy(bp.vz.begin(), bp.vz.end(), particles.vz.span().begin());
    std::copy(bp.inv_mass.begin(), bp.inv_mass.end(), particles.inv_mass.span().begin());
    std::copy(bp.pinned.begin(), bp.pinned.end(), particles.pinned.span().begin());

    if (bp.edge_i.empty()) return;

    auto dist = cloth.distance();
    std::copy(bp.edge_i.begin(), bp.edge_i.end(), dist.i.span().begin());
    std::copy(bp.edge_j.begin(), bp.edge_j.end(), dist.j.span().begin());
    std::copy(bp.rest.begin(), bp.rest.end(), dist.rest.span().begin());
    std::copy(bp.compliance.begin(), bp.compliance.end(), dist.compliance.span().begin());
    std::copy(bp.lambda.begin(), bp.lambda.end(), dist.lambda.span().begin());
    std::copy(bp.alpha.begin(), bp.alpha.end(), dist.alpha.span().begin());
    std::copy(bp.color.begin(), bp.color.end(), dist.color.span().begin());
}

ClothRenderer::Vec3 ClothRenderer::cross(const Vec3& a, const Vec3& b) {
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float ClothRenderer::dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

ClothRenderer::Vec3 ClothRenderer::subtract(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

ClothRenderer::Vec3 ClothRenderer::normalize(const Vec3& v) {
    const float len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if (len_sq <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
    const float inv = 1.0f / std::sqrt(len_sq);
    return Vec3{v.x * inv, v.y * inv, v.z * inv};
}

std::array<float, 16> ClothRenderer::make_look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
    Vec3 f = normalize(subtract(center, eye));
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);

    std::array<float, 16> m{};
    m[0] = s.x;  m[4] = s.y;  m[8] = s.z;   m[12] = -dot(s, eye);
    m[1] = u.x;  m[5] = u.y;  m[9] = u.z;   m[13] = -dot(u, eye);
    m[2] = -f.x; m[6] = -f.y; m[10] = -f.z; m[14] = dot(f, eye);
    m[3] = 0.0f; m[7] = 0.0f; m[11] = 0.0f; m[15] = 1.0f;
    return m;
}

std::array<float, 16> ClothRenderer::make_perspective(float fovy_radians, float aspect, float near_plane, float far_plane) {
    const float f = 1.0f / std::tan(fovy_radians * 0.5f);
    std::array<float, 16> m{};
    m[0] = f / aspect;
    m[5] = -f; // Flip Y for Vulkan clip space
    m[10] = (far_plane + near_plane) / (near_plane - far_plane);
    m[14] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    m[11] = -1.0f;
    return m;
}

std::array<float, 16> ClothRenderer::multiply(const std::array<float, 16>& a, const std::array<float, 16>& b) {
    std::array<float, 16> m{};
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a[k * 4 + row] * b[col * 4 + k];
            }
            m[col * 4 + row] = sum;
        }
    }
    return m;
}

void ClothRenderer::reset_simulation() {
    load_cloth(cloth_, blueprint_);
    accumulator_ = 0.0;
    pending_upload_ = true;
}


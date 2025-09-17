// XPBD Cloth Visualization with VulkanVisualizer (single-file example)
// - Implements an IRenderer to integrate with vulkan-visualizer
// - Simulates a simple XPBD cloth and visualizes particles + constraints
// - Uses ImGui draw lists for rendering primitives (no custom Vulkan pipelines)
// - Includes a lightweight 2D camera with pan/zoom and full control panel

#include "vk_engine.h"

#include <SDL3/SDL.h>
#include <imgui.h>
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef VK_CHECK
#define VK_CHECK(x)                                                                                                   \
    do {                                                                                                             \
        VkResult _res = (x);                                                                                         \
        if (_res != VK_SUCCESS) {                                                                                    \
            throw std::runtime_error(std::string("Vulkan error ") + std::to_string(_res) + " at " #x);             \
        }                                                                                                            \
    } while (false)
#endif

struct float2 {
    float x{0.0f};
    float y{0.0f};
};

static inline float2 operator+(const float2 &a, const float2 &b) { return {a.x + b.x, a.y + b.y}; }
static inline float2 operator-(const float2 &a, const float2 &b) { return {a.x - b.x, a.y - b.y}; }
static inline float2 operator*(const float2 &a, float s) { return {a.x * s, a.y * s}; }
static inline float2 operator*(float s, const float2 &a) { return {a.x * s, a.y * s}; }
static inline float2 operator/(const float2 &a, float s) { return {a.x / s, a.y / s}; }

static inline float dot(const float2 &a, const float2 &b) { return a.x * b.x + a.y * b.y; }
static inline float length(const float2 &a) { return std::sqrt(dot(a, a)); }
static inline float2 normalize(const float2 &a) {
    float l = length(a);
    if (l <= 1e-8f) return {0.0f, 0.0f};
    return a / l;
}

struct Particle {
    float2 x;       // current position
    float2 p;       // predicted position (for solver)
    float2 v;       // velocity
    float invm{1};  // inverse mass (0 -> pinned)
    bool pinned{false};
};

struct DistanceConstraint {
    uint32_t i{0};
    uint32_t j{0};
    float rest{0.0f};
    float lambda{0.0f}; // XPBD lagrange multiplier memory
};

struct Camera2D {
    // World-space center and pixels-per-unit scale
    float2 center{0.0f, 0.0f};
    float scale{100.0f}; // pixels per world unit

    // Convert world (sim) position to screen pixel position
    ImVec2 worldToScreen(const float2 &w, const ImVec2 &screenOrigin, const ImVec2 &screenSize) const {
        // Vulkan/ImGui screen coords: origin top-left, +Y downward
        // Sim coords: +Y upward. Apply Y flip here.
        float sx = screenOrigin.x + screenSize.x * 0.5f + (w.x - center.x) * scale;
        float sy = screenOrigin.y + screenSize.y * 0.5f - (w.y - center.y) * scale;
        return ImVec2(sx, sy);
    }

    // Adjust zoom while keeping an anchor (typically mouse position) stable
    void zoomAt(const float2 &anchorWorld, float zoomFactor) {
        scale = std::clamp(scale * zoomFactor, 5.0f, 5000.0f);
        (void)anchorWorld; // For simple 2D camera we keep center; could refine to keep anchor fixed
    }
};

class XpbdClothRenderer final : public IRenderer {
public:
    // ---- Engine lifecycle ----
    void initialize(const EngineContext &eng) override {
        (void)eng;
        resetCloth();
    }

    void destroy(const EngineContext &eng) override {
        destroyPipelines(eng.device);
        destroyBuffer(eng.device, eng.allocator, vbPoints_, vbPointsAlloc_);
        destroyBuffer(eng.device, eng.allocator, vbLines_, vbLinesAlloc_);
    }

    void get_capabilities(RendererCaps &caps) const override {
        caps = RendererCaps{}; // default: dynamic rendering, frames in flight, etc.
    }

    void on_swapchain_ready(const EngineContext &eng, const FrameContext &frm) override {
        (void)eng;
        viewportSize_ = ImVec2((float)frm.extent.width, (float)frm.extent.height);
        if (!cameraInitialized_) { cameraInitialized_ = true; cam_.center = {0.0f, 0.0f}; }
    }

    void on_swapchain_destroy(const EngineContext &eng) override { (void)eng; }

    // ---- Frame update & rendering ----
    void update(const EngineContext &, const FrameContext &frm) override {
        // Advance simulation with substeps
        if (!sim_.paused) {
            double dt = frm.dt_sec * sim_.timeScale;
            accumulator_ += dt;
            const double maxFrame = 1.0 / 20.0; // clamp long frames
            if (accumulator_ > 0.25) accumulator_ = 0.25;
            double step = std::max(1e-6, (double)sim_.dt);
            int maxSteps = (int)std::ceil(std::min(accumulator_, maxFrame) / step);
            int steps = std::min(maxSteps, sim_.maxSubsteps);
            for (int s = 0; s < steps; ++s) {
                simulateOnce((float)step);
                accumulator_ -= step;
            }
        }
    }

    void record_graphics(VkCommandBuffer cmd, const EngineContext &eng, const FrameContext &frm) override {
        ensurePipelines(eng.device, frm.swapchain_format);
        ensureBuffers(eng);

        uploadVertexData();

        // Transition offscreen to COLOR_ATTACHMENT
        VkImageMemoryBarrier2 toColor{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        toColor.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        toColor.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        toColor.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toColor.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT;
        toColor.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        toColor.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toColor.image = frm.offscreen_image;
        toColor.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkDependencyInfo dep{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dep.imageMemoryBarrierCount = 1; dep.pImageMemoryBarriers = &toColor;
        vkCmdPipelineBarrier2(cmd, &dep);

        // Begin dynamic rendering
        VkClearValue clear{}; clear.color = { { bgColor_.x, bgColor_.y, bgColor_.z, 1.0f } };
        VkRenderingAttachmentInfo att{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        att.imageView = frm.offscreen_image_view;
        att.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.clearValue = clear;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
        ri.renderArea = {{0, 0}, frm.extent};
        ri.layerCount = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments = &att;
        vkCmdBeginRendering(cmd, &ri);

        VkViewport vp{}; vp.x = 0; vp.y = 0; vp.width = (float)frm.extent.width; vp.height = (float)frm.extent.height; vp.minDepth = 0.f; vp.maxDepth = 1.f;
        VkRect2D sc{{0, 0}, frm.extent};
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);

        PushPC pc{};
        const float halfW = 0.5f * (float)frm.extent.width;
        const float halfH = 0.5f * (float)frm.extent.height;
        const float sx = cam_.scale / std::max(1.0f, halfW);
        const float sy = cam_.scale / std::max(1.0f, halfH);
        pc.scaleX = sx; pc.scaleY = -sy;
        pc.offsetX = -cam_.center.x * sx; pc.offsetY = cam_.center.y * sy;

        // Draw constraints as lines
        if (viz_.showEdges && linesCount_ > 0) {
            pc.pointSize = 1.0f;
            pc.color[0] = viz_.edgeColor.x; pc.color[1] = viz_.edgeColor.y; pc.color[2] = viz_.edgeColor.z; pc.color[3] = viz_.edgeColor.w;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeLines_);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &vbLines_, &off);
            vkCmdPushConstants(cmd, pipeLayout_, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushPC), &pc);
            vkCmdDraw(cmd, linesCount_, 1, 0, 0);
        }

        // Draw particles as points
        if (viz_.showPoints && pointsCount_ > 0) {
            pc.pointSize = viz_.pointRadiusPx * 2.0f;
            pc.color[0] = viz_.ptColor.x; pc.color[1] = viz_.ptColor.y; pc.color[2] = viz_.ptColor.z; pc.color[3] = viz_.ptColor.w;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipePoints_);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &vbPoints_, &off);
            vkCmdPushConstants(cmd, pipeLayout_, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushPC), &pc);
            vkCmdDraw(cmd, pointsCount_, 1, 0, 0);
        }

        vkCmdEndRendering(cmd);

        // Back to GENERAL for engine blit
        VkImageMemoryBarrier2 toGeneral{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        toGeneral.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toGeneral.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toGeneral.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        toGeneral.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
        toGeneral.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toGeneral.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        toGeneral.image = frm.offscreen_image;
        toGeneral.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkDependencyInfo dep2{VK_STRUCTURE_TYPE_DEPENDENCY_INFO}; dep2.imageMemoryBarrierCount = 1; dep2.pImageMemoryBarriers = &toGeneral;
        vkCmdPipelineBarrier2(cmd, &dep2);
    }

    // ---- Input & UI ----
    void on_event(const SDL_Event &e, const EngineContext &, const FrameContext *frm) override {
        (void)frm;
        switch (e.type) {
        case SDL_EVENT_MOUSE_WHEEL: {
            float scroll = (float)e.wheel.y;
            float factor = (scroll > 0) ? 1.1f : (scroll < 0 ? 1.0f / 1.1f : 1.0f);
            float2 mouseWorld = screenToWorld(ImGui::GetIO().MousePos);
            cam_.zoomAt(mouseWorld, factor);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (e.button.button == SDL_BUTTON_RIGHT) {
                input_.panning = true;
                input_.lastMouse = ImGui::GetIO().MousePos;
            }
            if (e.button.button == SDL_BUTTON_LEFT && sim_.enableDrag) {
                // Pick nearest particle within a radius in pixels
                input_.dragging = true;
                input_.dragIndex = pickParticle(ImGui::GetIO().MousePos, 16.0f);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: {
            if (e.button.button == SDL_BUTTON_RIGHT) input_.panning = false;
            if (e.button.button == SDL_BUTTON_LEFT) {
                input_.dragging = false;
                input_.dragIndex = UINT32_MAX;
            }
            break;
        }
        case SDL_EVENT_MOUSE_MOTION: {
            ImVec2 mpos = ImGui::GetIO().MousePos;
            if (input_.panning) {
                ImVec2 delta(mpos.x - input_.lastMouse.x, mpos.y - input_.lastMouse.y);
                cam_.center.x -= delta.x / cam_.scale;
                cam_.center.y += delta.y / cam_.scale; // invert to match world +Y up
                input_.lastMouse = mpos;
            }
            if (input_.dragging && input_.dragIndex != UINT32_MAX) {
                float2 w = screenToWorld(mpos);
                if (!particles_.empty() && input_.dragIndex < particles_.size() && !particles_[input_.dragIndex].pinned) {
                    particles_[input_.dragIndex].x = w;
                    particles_[input_.dragIndex].v = {0, 0};
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            // SDL3: use e.key.key (SDL_Keycode) or e.key.scancode (SDL_Scancode)
            if (e.key.key == SDLK_SPACE) sim_.paused = !sim_.paused;
            if (e.key.key == SDLK_R) resetCloth();
            if (e.key.key == SDLK_F) fitView();
            break;
        }
        default: break;
        }
    }

    void on_imgui(const EngineContext &, const FrameContext &frm) override {
        // Stats window
        ImGui::Begin("XPBD Cloth - Controls");
        ImGui::Text("Frame: %llu  (dt=%.3f ms)", (unsigned long long)frm.frame_index, frm.dt_sec * 1000.0);
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Paused", &sim_.paused);
            ImGui::SameLine();
            if (ImGui::Button("Step")) simulateOnce(sim_.dt);
            ImGui::SameLine();
            if (ImGui::Button("Reset")) resetCloth();

            ImGui::SliderFloat("dt (s)", &sim_.dt, 1e-4f, 0.033f, "%.5f", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("Time Scale", &sim_.timeScale, 0.0f, 2.0f, "%.2f");
            ImGui::SliderInt("Max Substeps", &sim_.maxSubsteps, 1, 64);
            ImGui::SliderInt("Solver Iterations", &sim_.iterations, 1, 80);
            ImGui::SliderFloat("Compliance", &sim_.compliance, 0.0f, 1e-2f, "%.6f", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("Damping", &sim_.damping, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat2("Gravity", &sim_.gravity.x, -50.0f, 50.0f, "%.2f");
            ImGui::Checkbox("Drag with Mouse", &sim_.enableDrag);
            if (ImGui::Button("Add Wind Impulse")) applyWindImpulse();

            ImGui::SeparatorText("Cloth Setup");
            bool changed = false;
            changed |= ImGui::SliderInt("Width (points)", &sim_.clothW, 2, 200);
            changed |= ImGui::SliderInt("Height (points)", &sim_.clothH, 2, 200);
            changed |= ImGui::SliderFloat("Spacing (units)", &sim_.spacing, 0.01f, 0.5f, "%.3f");
            if (changed) pendingRebuild_ = true;
            ImGui::Checkbox("Pin Top-Left", &sim_.pinLeft);
            ImGui::SameLine();
            ImGui::Checkbox("Pin Top-Right", &sim_.pinRight);
            ImGui::SameLine();
            if (ImGui::Button("Rebuild Cloth")) { resetCloth(); pendingRebuild_ = false; }
        }

        if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Background", (float *)&bgColor_);
            ImGui::ColorEdit3("Point Color", (float *)&viz_.ptColor);
            ImGui::ColorEdit3("Constraint Color", (float *)&viz_.edgeColor);
            ImGui::SliderFloat("Point Size (px)", &viz_.pointRadiusPx, 1.0f, 12.0f, "%.1f");
            ImGui::SliderFloat("Line Thickness", &viz_.lineThickness, 1.0f, 4.0f, "%.1f");
            ImGui::Checkbox("Show Constraints", &viz_.showEdges);
            ImGui::Checkbox("Show Points", &viz_.showPoints);
        }

        if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Center: (%.2f, %.2f)", cam_.center.x, cam_.center.y);
            ImGui::Text("Scale: %.1f px/unit", cam_.scale);
            if (ImGui::Button("Reset Camera")) resetCamera();
            ImGui::SameLine();
            if (ImGui::Button("Fit View")) fitView();
        }
        ImGui::End();

        // Apply pending rebuild lazily (after UI interaction)
        if (pendingRebuild_) {
            resetCloth();
            pendingRebuild_ = false;
        }

        // Draw constraints & points on the background draw list (full-screen)
        drawScene();
    }

private:
    // ---- Simulation core (XPBD) ----
    void resetCloth() {
        particles_.clear();
        constraints_.clear();

        const int W = std::max(2, sim_.clothW);
        const int H = std::max(2, sim_.clothH);
        particles_.resize((size_t)W * (size_t)H);

        // Center the cloth in the current camera view
        float halfW = 0.5f * (W - 1) * sim_.spacing;
        float halfH = 0.5f * (H - 1) * sim_.spacing;
        float2 origin{cam_.center.x - halfW, cam_.center.y + halfH}; // top-left corner in world space
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                size_t idx = (size_t)y * (size_t)W + (size_t)x;
                particles_[idx].x = {origin.x + x * sim_.spacing, origin.y - y * sim_.spacing};
                particles_[idx].v = {0, 0};
                particles_[idx].pinned = false;
                particles_[idx].invm = 1.0f;
            }
        }

        auto addEdge = [&](int x0, int y0, int x1, int y1) {
            size_t i = (size_t)y0 * (size_t)W + (size_t)x0;
            size_t j = (size_t)y1 * (size_t)W + (size_t)x1;
            float2 d = particles_[i].x - particles_[j].x;
            float r = length(d);
            constraints_.push_back(DistanceConstraint{(uint32_t)i, (uint32_t)j, r, 0.0f});
        };

        // Structural constraints (horizontal + vertical)
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if (x + 1 < W) addEdge(x, y, x + 1, y);
                if (y + 1 < H) addEdge(x, y, x, y + 1);
            }
        }

        // Pin top-left and top-right if enabled
        particles_[0 * W + 0].pinned = sim_.pinLeft;
        particles_[0 * W + 0].invm = sim_.pinLeft ? 0.0f : 1.0f;
        particles_[0 * W + (W - 1)].pinned = sim_.pinRight;
        particles_[0 * W + (W - 1)].invm = sim_.pinRight ? 0.0f : 1.0f;

        // Keep camera as-is; the cloth is already centered on cam_.center
    }

    void simulateOnce(float dt) {
        // Semi-implicit Euler to predict positions
        float damp = std::clamp(sim_.damping, 0.0f, 1.0f);
        for (auto &p : particles_) {
            if (p.invm <= 0.0f) {
                p.p = p.x; // pinned
                continue;
            }
            p.v = (p.v + sim_.gravity * dt) * (1.0f - damp);
            p.p = p.x + p.v * dt;
        }

        float alpha = sim_.compliance / (dt * dt);
        for (int it = 0; it < sim_.iterations; ++it) {
            for (auto &c : constraints_) {
                Particle &pi = particles_[c.i];
                Particle &pj = particles_[c.j];
                float wi = pi.invm;
                float wj = pj.invm;
                if (wi == 0.0f && wj == 0.0f) continue;

                float2 d = pi.p - pj.p;
                float len = length(d);
                if (len < 1e-8f) continue;
                float C = len - c.rest;
                float2 n = d / len;
                float denom = wi + wj + alpha;
                float dlambda = (-C - alpha * c.lambda) / denom;
                float2 corr = n * dlambda;
                if (wi > 0.0f) pi.p = pi.p + corr * wi;
                if (wj > 0.0f) pj.p = pj.p - corr * wj;
                c.lambda += dlambda;
            }
        }

        // Update velocities and positions
        for (auto &p : particles_) {
            if (p.invm <= 0.0f) continue;
            p.v = (p.p - p.x) / dt;
            p.x = p.p;
        }

        // Reset lambdas each step (XPBD keeps them if compliance > 0)
        if (sim_.compliance <= 0.0f) {
            for (auto &c : constraints_) c.lambda = 0.0f;
        }
    }

    void applyWindImpulse() {
        float2 w{sim_.wind.x, sim_.wind.y};
        for (auto &p : particles_) {
            if (p.invm > 0.0f) p.v = p.v + w;
        }
    }

    // ---- Rendering via ImGui draw lists ----
    void drawScene() { /* now rendered via Vulkan pipeline */ }

    // ---- Camera helpers ----
    void resetCamera() {
        cam_.center = {0.0f, 0.0f};
        cam_.scale = 120.0f;
    }

    void fitView() {
        if (particles_.empty()) return;
        float2 minp{1e9f, 1e9f}, maxp{-1e9f, -1e9f};
        for (const auto &p : particles_) {
            minp.x = std::min(minp.x, p.x.x);
            minp.y = std::min(minp.y, p.x.y);
            maxp.x = std::max(maxp.x, p.x.x);
            maxp.y = std::max(maxp.y, p.x.y);
        }
        float2 c{0.5f * (minp.x + maxp.x), 0.5f * (minp.y + maxp.y)};
        cam_.center = c;
        float2 ext{maxp.x - minp.x, maxp.y - minp.y};
        float padding = 40.0f; // pixels
        float sx = (viewportSize_.x - padding) / std::max(0.1f, ext.x);
        float sy = (viewportSize_.y - padding) / std::max(0.1f, ext.y);
        cam_.scale = std::clamp(std::min(sx, sy), 10.0f, 5000.0f);
    }

    // Convert from screen (pixels) to world (sim) position
    float2 screenToWorld(const ImVec2 &sp) const {
        float2 w{};
        w.x = (sp.x - viewportSize_.x * 0.5f) / cam_.scale + cam_.center.x;
        w.y = (viewportSize_.y * 0.5f - sp.y) / cam_.scale + cam_.center.y;
        return w;
    }

    uint32_t pickParticle(const ImVec2 &sp, float radiusPx) const {
        if (particles_.empty()) return UINT32_MAX;
        float rr = radiusPx * radiusPx;
        float bestD = 1e20f;
        uint32_t best = UINT32_MAX;
        for (uint32_t i = 0; i < particles_.size(); ++i) {
            ImVec2 pp = cam_.worldToScreen(particles_[i].x, ImVec2(0, 0), viewportSize_);
            float dx = pp.x - sp.x, dy = pp.y - sp.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < rr && d2 < bestD) { bestD = d2; best = i; }
        }
        return best;
    }

private:
    struct SimParams {
        // Cloth grid
        int clothW{30};
        int clothH{20};
        float spacing{0.05f};
        bool pinLeft{true};
        bool pinRight{true};

        // Dynamics
        float2 gravity{0.0f, -9.81f};
        float dt{1.0f / 240.0f};
        int maxSubsteps{16};
        float timeScale{1.0f};
        int iterations{20};
        float compliance{0.0f}; // 0 -> PBD, >0 -> XPBD soft constraints
        float damping{0.01f};
        bool paused{false};
        bool enableDrag{true};
        float2 wind{2.0f, 0.0f};
    } sim_{};

    struct VizParams {
        bool showEdges{true};
        bool showPoints{true};
        float pointRadiusPx{3.0f};
        float lineThickness{1.5f};
        ImVec4 ptColor{0.9f, 0.9f, 0.95f, 1.0f};
        ImVec4 edgeColor{0.5f, 0.8f, 1.0f, 1.0f};
    } viz_{};

    Camera2D cam_{};
    bool cameraInitialized_{false};
    ImVec2 viewportSize_{1280, 720};
    ImVec4 bgColor_{0.06f, 0.07f, 0.09f, 1.0f};

    std::vector<Particle> particles_{};
    std::vector<DistanceConstraint> constraints_{};

    struct {
        bool panning{false};
        ImVec2 lastMouse{0, 0};
        bool dragging{false};
        uint32_t dragIndex{UINT32_MAX};
    } input_{};

    double accumulator_{0.0};

    bool pendingRebuild_{false};

    // --- GPU resources ---
    struct PushPC {
        float scaleX, scaleY;
        float offsetX, offsetY;
        float pointSize;
        float pad0, pad1, pad2; // align to 16 bytes
        float color[4];
    };

    VkPipeline pipePoints_{VK_NULL_HANDLE};
    VkPipeline pipeLines_{VK_NULL_HANDLE};
    VkPipelineLayout pipeLayout_{VK_NULL_HANDLE};

    VkBuffer vbPoints_{VK_NULL_HANDLE};
    VmaAllocation vbPointsAlloc_{nullptr};
    void *vbPointsMapped_{nullptr};
    uint32_t pointsCapacity_{0};
    uint32_t pointsCount_{0};

    VkBuffer vbLines_{VK_NULL_HANDLE};
    VmaAllocation vbLinesAlloc_{nullptr};
    void *vbLinesMapped_{nullptr};
    uint32_t linesCapacity_{0};
    uint32_t linesCount_{0};

    void ensurePipelines(VkDevice device, VkFormat colorFormat) {
        if (pipePoints_ && pipeLines_ && currentColorFormat_ == colorFormat) return;
        destroyPipelines(device);
        currentColorFormat_ = colorFormat;

        auto load_file = [](const std::string &path) {
            std::ifstream f(path, std::ios::binary | std::ios::ate);
            if (!f) throw std::runtime_error("Failed to open file: " + path);
            size_t sz = (size_t)f.tellg(); f.seekg(0);
            std::vector<char> data(sz); f.read(data.data(), sz); return data;
        };

        auto shader_path = [](const char *name) {
            std::string base = std::string(SHADER_OUTPUT_DIR);
            std::string p = base + "/" + name;
            std::ifstream test(p, std::ios::binary);
            if (test.good()) return p;
            return std::string(SHADER_SOURCE_DIR) + "/" + name; // fallback
        };

        auto make_shader = [&](const char *fn) {
            auto code = load_file(shader_path(fn));
            VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
            ci.codeSize = code.size();
            ci.pCode = reinterpret_cast<const uint32_t *>(code.data());
            VkShaderModule mod{}; VK_CHECK(vkCreateShaderModule(device, &ci, nullptr, &mod));
            return mod;
        };

        VkShaderModule vs = make_shader("xpbd.vert.spv");
        VkShaderModule fs = make_shader("xpbd.frag.spv");

        VkPushConstantRange pcr{}; pcr.offset = 0; pcr.size = sizeof(PushPC); pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plci.pushConstantRangeCount = 1; plci.pPushConstantRanges = &pcr; VK_CHECK(vkCreatePipelineLayout(device, &plci, nullptr, &pipeLayout_));

        auto create_pipeline = [&](VkPrimitiveTopology topo) {
            VkPipelineShaderStageCreateInfo stages[2]{};
            stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT; stages[0].module = vs; stages[0].pName = "main";
            stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module = fs; stages[1].pName = "main";

            VkVertexInputBindingDescription bind{}; bind.binding = 0; bind.stride = sizeof(float2); bind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            VkVertexInputAttributeDescription attr{}; attr.location = 0; attr.binding = 0; attr.format = VK_FORMAT_R32G32_SFLOAT; attr.offset = 0;
            VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
            vi.vertexBindingDescriptionCount = 1; vi.pVertexBindingDescriptions = &bind; vi.vertexAttributeDescriptionCount = 1; vi.pVertexAttributeDescriptions = &attr;

            VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO}; ia.topology = topo; ia.primitiveRestartEnable = VK_FALSE;
            VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO}; vp.viewportCount = 1; vp.scissorCount = 1;
            VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO}; rs.polygonMode = VK_POLYGON_MODE_FILL; rs.cullMode = VK_CULL_MODE_NONE; rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE; rs.lineWidth = 1.0f;
            VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
            VkPipelineColorBlendAttachmentState cbatt{}; cbatt.colorWriteMask = 0xF;
            VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; cb.attachmentCount = 1; cb.pAttachments = &cbatt;
            VkDynamicState dynArr[2]{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO}; dyn.dynamicStateCount = 2; dyn.pDynamicStates = dynArr;

            VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO}; rendering.colorAttachmentCount = 1; rendering.pColorAttachmentFormats = &currentColorFormat_;
            VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            gp.pNext = &rendering; gp.stageCount = 2; gp.pStages = stages; gp.pVertexInputState = &vi; gp.pInputAssemblyState = &ia; gp.pViewportState = &vp; gp.pRasterizationState = &rs; gp.pMultisampleState = &ms; gp.pColorBlendState = &cb; gp.pDynamicState = &dyn; gp.layout = pipeLayout_; gp.renderPass = VK_NULL_HANDLE;
            VkPipeline pipe{}; VK_CHECK(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gp, nullptr, &pipe));
            return pipe;
        };

        pipePoints_ = create_pipeline(VK_PRIMITIVE_TOPOLOGY_POINT_LIST);
        pipeLines_ = create_pipeline(VK_PRIMITIVE_TOPOLOGY_LINE_LIST);

        vkDestroyShaderModule(device, vs, nullptr);
        vkDestroyShaderModule(device, fs, nullptr);
    }

    void destroyPipelines(VkDevice device) {
        if (pipePoints_) { vkDestroyPipeline(device, pipePoints_, nullptr); pipePoints_ = VK_NULL_HANDLE; }
        if (pipeLines_) { vkDestroyPipeline(device, pipeLines_, nullptr); pipeLines_ = VK_NULL_HANDLE; }
        if (pipeLayout_) { vkDestroyPipelineLayout(device, pipeLayout_, nullptr); pipeLayout_ = VK_NULL_HANDLE; }
    }

    void ensureBuffers(const EngineContext &eng) {
        // Points buffer
        uint32_t neededPts = (uint32_t)particles_.size();
        if (neededPts > pointsCapacity_) {
            destroyBuffer(eng.device, eng.allocator, vbPoints_, vbPointsAlloc_);
            createBuffer(eng.allocator, neededPts * sizeof(float2), vbPoints_, vbPointsAlloc_, &vbPointsMapped_);
            pointsCapacity_ = neededPts;
        }
        // Lines buffer: 2 vertices per constraint
        uint32_t neededLines = (uint32_t)constraints_.size() * 2u;
        if (neededLines > linesCapacity_) {
            destroyBuffer(eng.device, eng.allocator, vbLines_, vbLinesAlloc_);
            createBuffer(eng.allocator, neededLines * sizeof(float2), vbLines_, vbLinesAlloc_, &vbLinesMapped_);
            linesCapacity_ = neededLines;
        }
    }

    void createBuffer(VmaAllocator allocator, VkDeviceSize size, VkBuffer &buf, VmaAllocation &alloc, void **mapped) {
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; bci.size = size; bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_AUTO; aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo ai{}; VK_CHECK(vmaCreateBuffer(allocator, &bci, &aci, &buf, &alloc, &ai));
        *mapped = ai.pMappedData;
    }

    void destroyBuffer(VkDevice device, VmaAllocator allocator, VkBuffer &buf, VmaAllocation &alloc) {
        if (buf != VK_NULL_HANDLE) { vmaDestroyBuffer(allocator, buf, alloc); buf = VK_NULL_HANDLE; alloc = nullptr; }
    }

    void uploadVertexData() {
        // Points
        pointsCount_ = (uint32_t)particles_.size();
        if (vbPointsMapped_ && pointsCount_ > 0) {
            auto *dst = reinterpret_cast<float2 *>(vbPointsMapped_);
            for (uint32_t i = 0; i < pointsCount_; ++i) dst[i] = particles_[i].x;
        }
        // Lines
        linesCount_ = (uint32_t)constraints_.size() * 2u;
        if (vbLinesMapped_ && linesCount_ > 0) {
            auto *dst = reinterpret_cast<float2 *>(vbLinesMapped_);
            uint32_t k = 0;
            for (const auto &c : constraints_) {
                dst[k++] = particles_[c.i].x;
                dst[k++] = particles_[c.j].x;
            }
        }
    }

    VkFormat currentColorFormat_{VK_FORMAT_UNDEFINED};
};

int main() {
    try {
        VulkanEngine engine;
        engine.state_.name = "XPBD Cloth (VulkanVisualizer + ImGui)";
        engine.state_.width = 1600;
        engine.state_.height = 900;
        engine.set_renderer(std::make_unique<XpbdClothRenderer>());
        engine.init();
        engine.run();
        engine.cleanup();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Fatal error: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

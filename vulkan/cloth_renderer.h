#ifndef HINACLOTH_CLOTH_RENDERER_H
#define HINACLOTH_CLOTH_RENDERER_H

#include "vk_engine.h"

#include "cloth_data.h"
#include "xpbd.h"

#include <array>
#include <memory>
#include <vector>

class ClothRenderer : public IRenderer {
public:
    ClothRenderer();
    ~ClothRenderer() override;

    void initialize(const EngineContext& eng) override;
    void destroy(const EngineContext& eng) override;

    void on_swapchain_ready(const EngineContext& eng, const FrameContext& frm) override;
    void on_swapchain_destroy(const EngineContext& eng) override;

    void update(const EngineContext& eng, const FrameContext& frm) override;
    void record_graphics(VkCommandBuffer cmd, const EngineContext& eng, const FrameContext& frm) override;

    void on_event(const SDL_Event& e, const EngineContext& eng, const FrameContext* frm) override;
    void on_imgui(const EngineContext& eng, const FrameContext& frm) override;

    void reload_assets(const EngineContext& eng) override;
    void get_capabilities(RendererCaps& out_caps) const override;
    RendererStats get_stats() const override;

private:
    struct HostBuffer {
        VkBuffer buffer{VK_NULL_HANDLE};
        VmaAllocation allocation{VK_NULL_HANDLE};
        void* mapped{nullptr};
        VkDeviceSize size{0};
    };

    struct Vertex {
        float position[3];
        float color[3];
    };

    struct ClothBlueprint {
        uint32_t width{0};
        uint32_t height{0};
        float spacing{0.025f};
        std::vector<float> px, py, pz;
        std::vector<float> vx, vy, vz;
        std::vector<float> inv_mass;
        std::vector<HinaPE::u8> pinned;
        std::vector<HinaPE::u32> edge_i, edge_j;
        std::vector<float> rest, compliance, lambda, alpha;
        std::vector<HinaPE::u8> color;
    };

    struct PushConstants {
        std::array<float, 16> mvp{};
        float point_size{4.0f};
        float line_width{1.0f};
        float padding0{0.0f};
        float padding1{0.0f};
    };

    struct Vec3 {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    void destroy_buffers(const EngineContext& eng);
    void destroy_pipeline(const EngineContext& eng);

    void create_pipeline(const EngineContext& eng, VkFormat swapchain_format);
    void ensure_buffers(const EngineContext& eng);

    void upload_frame_buffers(uint32_t frame_index);
    void build_vertices();

    static ClothBlueprint make_grid_blueprint(uint32_t width, uint32_t height, float spacing);
    static void load_cloth(HinaPE::ClothData& cloth, const ClothBlueprint& bp);

    static Vec3 cross(const Vec3& a, const Vec3& b);
    static Vec3 subtract(const Vec3& a, const Vec3& b);
    static Vec3 normalize(const Vec3& v);
    static float dot(const Vec3& a, const Vec3& b);

    static std::array<float, 16> make_look_at(const Vec3& eye, const Vec3& center, const Vec3& up);
    static std::array<float, 16> make_perspective(float fovy_radians, float aspect, float near_plane, float far_plane);
    static std::array<float, 16> multiply(const std::array<float, 16>& a, const std::array<float, 16>& b);

    void reset_simulation();

    EngineContext eng_{};
    bool initialized_{false};
    bool pipelines_ready_{false};
    VkFormat swapchain_format_{VK_FORMAT_UNDEFINED};
    bool offscreen_ready_{false};

    VkShaderModule vert_module_{VK_NULL_HANDLE};
    VkShaderModule frag_module_{VK_NULL_HANDLE};
    VkPipelineLayout pipeline_layout_{VK_NULL_HANDLE};
    VkPipeline line_pipeline_{VK_NULL_HANDLE};
    VkPipeline point_pipeline_{VK_NULL_HANDLE};

    std::array<HostBuffer, FRAME_OVERLAP> point_buffers_{};
    std::array<HostBuffer, FRAME_OVERLAP> line_buffers_{};

    std::vector<Vertex> cpu_points_;
    std::vector<Vertex> cpu_lines_;

    size_t point_count_{0};
    size_t line_vertex_count_{0};

    HinaPE::ClothData cloth_;
    ClothBlueprint blueprint_;
    HinaPE::XPBDParams params_{};

    double accumulator_{0.0};
    bool pending_upload_{true};
    bool simulate_{true};
    bool draw_vertices_{true};
    bool draw_constraints_{true};

    float point_size_pixels_{6.0f};
    float line_width_pixels_{1.5f};

    Vec3 target_{0.0f, 0.0f, 0.0f};
    float camera_distance_{3.0f};
    float camera_yaw_{-1.1f};
    float camera_pitch_{-0.5f};

    bool rotating_{false};
    Sint32 last_mouse_x_{0};
    Sint32 last_mouse_y_{0};

    RendererStats stats_{};
};

std::unique_ptr<IRenderer> CreateClothRenderer();

#endif // HINACLOTH_CLOTH_RENDERER_H

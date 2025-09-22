/*
 * File: types.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_ENGINE_TYPES_H
#define HINACLOTH_ENGINE_TYPES_H
#include <cstddef>
#include <cstdint>

namespace sim { namespace eng {
    enum class Status {
        Ok,
        InvalidArgs,
        ValidationFailed,
        NoBackend,
        Unsupported,
        OOM,
        NotReady,
        Busy
    };

    struct TelemetryFrame {
        double   step_ms{};
        double   residual_avg{};
        double   last_rebuild_ms{};
        double   avg_rebuild_ms{};
        uint64_t commands_applied{};
        uint64_t structural_rebuilds{};
        int      solve_substeps{};
        int      solve_iterations{};
    };

    enum class DataLayout { Auto, SoA, AoS, Blocked };
    enum class Backend { Auto, Native, AVX2, TBB, GPU };
    enum class TimeStepper { Auto, Symplectic, SemiImplicit, Explicit };

    struct PolicyExec {
        DataLayout layout{DataLayout::Auto};
        Backend backend{Backend::Auto};
        int threads{0};
        bool deterministic{false};
        bool telemetry{false};
    };
    struct PolicySolve {
        int   substeps{1};
        int   iterations{8};
        float damping{0.0f};
        TimeStepper stepper{TimeStepper::Auto};
    };

    struct Capability {
        Backend backend;
        DataLayout layout;
        const char* name;
    };

    enum class ParamType { F32, I32, U32, BOOL, VEC2, VEC3, VEC4, MAT3, MAT4, STRING, BLOB };
    struct ParamVec2 { float x, y; };
    struct ParamVec3 { float x, y, z; };
    struct ParamVec4 { float x, y, z, w; };
    struct ParamMat3 { float m[9]; };
    struct ParamMat4 { float m[16]; };
    struct ParamBlob { const void* data; size_t bytes; };
    union ParamValue {
        float f32; int32_t i32; uint32_t u32; int32_t b;
        ParamVec2 vec2; ParamVec3 vec3; ParamVec4 vec4;
        ParamMat3 mat3; ParamMat4 mat4;
        const char* str; ParamBlob blob;
    };
    struct Param { const char* name; ParamType type; ParamValue value; };
    struct Parameters { const Param* items; size_t count; };

    enum class FieldType { F32, I32, U32 };
    struct FieldView {
        const char* name; FieldType type; const void* data; size_t count; size_t components; size_t stride_bytes;
    };
    struct StateInit { const FieldView* fields; size_t field_count; };

    struct RelationView { const uint32_t* indices; size_t arity; size_t count; const char* tag; };
    struct TopologyIn { uint32_t node_count; const RelationView* relations; size_t relation_count; };

    enum class ValidateLevel { Strict, Tolerant };
    struct PackOptions { bool lazy_pack{false}; int block_size{0}; };
    struct SpaceDesc { int dummy{0}; };
    struct OperatorsDecl { int dummy{0}; };
    struct EventsScript { int dummy{0}; };
    struct Policy { PolicyExec exec; PolicySolve solve; };

    struct BuildDesc {
        StateInit   state;
        Parameters  params;
        TopologyIn  topo;
        Policy      policy;
        SpaceDesc   space;
        OperatorsDecl ops;
        EventsScript  events;
        ValidateLevel validate{ValidateLevel::Strict};
        PackOptions pack{};
    };

    enum class ApplyPhase { BeforeFrame, AfterSolve };
    enum class CommandTag { SetParam, EnableOperator, DisableOperator, AddNodes, RemoveNodes, AddRelations, RemoveRelations, SetFieldRegion, Custom };
    struct Command { CommandTag tag; const void* data; size_t bytes; };

    struct Chosen { DataLayout layout; Backend backend; int threads; };

    struct SolveOverrides { int substeps_override{0}; int iterations_override{0}; };
}}

#endif

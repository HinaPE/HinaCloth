/*
 * File: engine_adapter.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_ENGINE_ADAPTER_H
#define HINACLOTH_ENGINE_ADAPTER_H
#include "api/build.h"
#include "api/commands.h"
#include "api/status.h"
#include "api/telemetry.h"
#include "api/chosen.h"

namespace sim {
    struct EngineHandle;

    struct SolveOverrides {
        int substeps_override;
        int iterations_override;
    };

    [[nodiscard]] EngineHandle* engine_create(const BuildDesc& desc);
    void engine_destroy(EngineHandle* e);
    Status engine_apply_small_params(EngineHandle* e, const Command* cmds, size_t count);
    Status engine_apply_structural_changes(EngineHandle* e, const Command* cmds, size_t count);
    Status engine_step(EngineHandle* e, float dt, const SolveOverrides* ovr, TelemetryFrame* out);
    [[nodiscard]] bool engine_query_chosen(EngineHandle* e, struct Chosen& out);
    Status engine_copy_positions(EngineHandle* e, float* dst, size_t maxCount, size_t* outCount);
}
#endif

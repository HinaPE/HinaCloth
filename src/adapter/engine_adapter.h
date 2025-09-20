#ifndef HINACLOTH_ENGINE_ADAPTER_H
#define HINACLOTH_ENGINE_ADAPTER_H
#include "api/build.h"
#include "api/commands.h"
#include "api/status.h"
#include "api/telemetry.h"

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
    // New: query actual chosen combo
    [[nodiscard]] bool engine_query_chosen(EngineHandle* e, struct Chosen& out);

    // New: copy current positions (x,y,z) into an interleaved float3 array.
    // - dst: pointer to at least 3*outCount floats (if outCount is known) or 3*maxCount
    // - maxCount: maximum number of vertices to copy (0 to copy all)
    // - outCount: returns actual number of vertices copied (may be null)
    // Returns Status::Ok on success, Status::InvalidArgs on bad params.
    Status engine_copy_positions(EngineHandle* e, float* dst, size_t maxCount, size_t* outCount);
}
#endif //HINACLOTH_ENGINE_ADAPTER_H

#ifndef HINACLOTH_COOKING_H
#define HINACLOTH_COOKING_H
#include <cstddef>
#include "core/model/model.h"
#include "core/data/remap.h"
#include "api/build.h"
#include "api/commands.h"

namespace sim {
    [[nodiscard]] bool cooking_build_model(const BuildDesc& in, Model*& out);
    [[nodiscard]] bool cooking_rebuild_model_from_commands(const Model& cur, const Command* cmds, std::size_t count, Model*& out, RemapPlan*& plan);
}
#endif //HINACLOTH_COOKING_H

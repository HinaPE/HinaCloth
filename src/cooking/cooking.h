/*
 * File: cooking.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_COOKING_H
#define HINACLOTH_COOKING_H
#include <cstddef>
#include "core/model/model.h"
#include "core/data/remap.h"
#include "core/common/types.h"

namespace sim {
    namespace eng { struct BuildDesc; struct Command; }
    [[nodiscard]] bool cooking_build_model(const eng::BuildDesc& in, Model*& out);
    [[nodiscard]] bool cooking_rebuild_model_from_commands(const Model& cur, const eng::Command* cmds, std::size_t count, Model*& out, RemapPlan*& plan);
}
#endif

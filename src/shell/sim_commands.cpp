#include "solver_internal.h"
#include <chrono>

namespace sim {
    static bool is_structural(const Command& c) {
        return c.tag == CommandTag::AddNodes
               || c.tag == CommandTag::RemoveNodes
               || c.tag == CommandTag::AddRelations
               || c.tag == CommandTag::RemoveRelations;
    }

    Status push_command(Solver* s, const Command& c) {
        if (!s) return Status::InvalidArgs;
        s->before.push_back(c);
        return Status::Ok;
    }

    static void update_rebuild_telemetry(Solver* s, double ms) {
        s->tf.last_rebuild_ms = ms;
        // simple running average
        if (s->rebuilds == 0) s->tf.avg_rebuild_ms = ms;
        else s->tf.avg_rebuild_ms = 0.9 * s->tf.avg_rebuild_ms + 0.1 * ms;
    }

    Status flush_commands(Solver* s, ApplyPhase p) {
        if (!s) return Status::InvalidArgs;
        if (p == ApplyPhase::BeforeFrame) {
            if (!s->before.empty()) {
                size_t n                 = s->before.size();
                std::vector<Command> tmp = s->before;
                s->before.clear();
                std::vector<Command> small;
                std::vector<Command> structural;
                small.reserve(tmp.size());
                structural.reserve(tmp.size());
                for (auto& c : tmp) {
                    if (is_structural(c)) structural.push_back(c);
                    else small.push_back(c);
                }
                if (!small.empty()) engine_apply_small_params(s->e, small.data(), small.size());
                if (!structural.empty()) {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    engine_apply_structural_changes(s->e, structural.data(), structural.size());
                    auto t1 = std::chrono::high_resolution_clock::now();
                    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                    s->rebuilds += 1;
                    update_rebuild_telemetry(s, ms);
                }
                s->applied += (unsigned long long) n;
                s->tf.commands_applied    = s->applied;
                s->tf.structural_rebuilds = s->rebuilds;
            }
        } else {
            if (!s->after.empty()) {
                size_t n                 = s->after.size();
                std::vector<Command> tmp = s->after;
                s->after.clear();
                std::vector<Command> small;
                std::vector<Command> structural;
                small.reserve(tmp.size());
                structural.reserve(tmp.size());
                for (auto& c : tmp) {
                    if (is_structural(c)) structural.push_back(c);
                    else small.push_back(c);
                }
                if (!small.empty()) engine_apply_small_params(s->e, small.data(), small.size());
                if (!structural.empty()) {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    engine_apply_structural_changes(s->e, structural.data(), structural.size());
                    auto t1 = std::chrono::high_resolution_clock::now();
                    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                    s->rebuilds += 1;
                    update_rebuild_telemetry(s, ms);
                }
                s->applied += (unsigned long long) n;
                s->tf.commands_applied    = s->applied;
                s->tf.structural_rebuilds = s->rebuilds;
            }
        }
        return Status::Ok;
    }
}

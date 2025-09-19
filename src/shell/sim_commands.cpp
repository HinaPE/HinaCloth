#include "solver_internal.h"

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
                    engine_apply_structural_changes(s->e, structural.data(), structural.size());
                    s->rebuilds += 1;
                }
                s->applied += (unsigned long long) n;
                s->tf.commands_applied    = s->applied;
                s->tf.structural_rebuilds = s->rebuilds;
            }
        } else {
            if (!s->after.empty()) {
                s->applied += (unsigned long long) s->after.size();
                s->after.clear();
                s->tf.commands_applied = s->applied;
            }
        }
        return Status::Ok;
    }
}

#include "api/build.h"
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace sim {
    static const FieldView* find_field_exact(const StateInit& s, const char* name) {
        for (size_t i = 0; i < s.field_count; ++i) if (s.fields[i].name && std::strcmp(s.fields[i].name, name) == 0) return &s.fields[i];
        return nullptr;
    }
    static const FieldView* find_field_any(const StateInit& s, const char* const* names, size_t count) {
        for (size_t k = 0; k < count; ++k) {
            if (auto* f = find_field_exact(s, names[k])) return f;
        }
        return nullptr;
    }

    static bool check_state(const StateInit& s, const TopologyIn& topo, ValidateLevel lvl) {
        if (s.field_count == 0 || !s.fields) return false;
        for (size_t i = 0; i < s.field_count; i++) {
            auto& f = s.fields[i];
            if (!f.name || !f.data) return false;
            if (f.count == 0 || f.components == 0) return false;
            if (f.stride_bytes == 0) return false;
            if (f.stride_bytes < f.components * sizeof(float)) {
                if (lvl == ValidateLevel::Strict) return false;
            }
        }
        // position required (accept aliases)
        const char* pos_aliases[] = {"position", "pos", "positions"};
        auto pos = find_field_any(s, pos_aliases, 3);
        if (!pos) return false;
        if (pos->components != 3) return false;
        if (pos->count != topo.node_count) return false;
        // velocity optional; if present must match (accept aliases)
        const char* vel_aliases[] = {"velocity", "vel", "velocities"};
        auto vel = find_field_any(s, vel_aliases, 3);
        if (vel) {
            if (vel->components != 3) return false;
            if (vel->count != topo.node_count) return false;
        }
        return true;
    }

    static bool check_topology(const TopologyIn& t, ValidateLevel lvl) {
        if (t.node_count == 0) return false;
        if (t.relation_count > 0 && !t.relations) return false;
        for (size_t i = 0; i < t.relation_count; ++i) {
            auto& r = t.relations[i];
            if (r.tag && std::strcmp(r.tag, "edges") == 0) {
                if (r.arity != 2) return false;
            } else if (r.tag && std::strcmp(r.tag, "bend_pairs") == 0) {
                if (r.arity != 4) return false;
            } else {
                // unknown relation tag: strict fails, tolerant ignores
                if (lvl == ValidateLevel::Strict) return false;
            }
        }
        return true;
    }

    static bool check_policy(const Policy& p) {
        if (p.solve.substeps < 0) return false;
        if (p.solve.iterations < 0) return false;
        return true;
    }

    bool shell_validate(const BuildDesc& d) {
        if (!check_policy(d.policy)) return false;
        if (!check_topology(d.topo, d.validate)) return false;
        if (!check_state(d.state, d.topo, d.validate)) return false;
        return true;
    }
}

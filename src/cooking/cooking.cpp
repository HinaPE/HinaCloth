#include "cooking.h"

#include <cstdint>
#include <cstring>
#include <new>
#include <vector>
#include <cmath>

namespace sim {
    static const void* find_field(const StateInit& st, const char* name, size_t comps, size_t& count, size_t& stride) {
        for (size_t i = 0; i < st.field_count; i++) {
            auto& f = st.fields[i];
            if (std::strcmp(f.name, name) == 0 && f.components == comps) {
                count  = f.count;
                stride = f.stride_bytes;
                return f.data;
            }
        }
        return nullptr;
    }

    static void load_vec3_aos(const void* src, size_t count, size_t stride, std::vector<float>& a, std::vector<float>& b, std::vector<float>& c) {
        a.resize(count);
        b.resize(count);
        c.resize(count);
        const char* p = (const char*) src;
        for (size_t i = 0; i < count; i++) {
            const float* v = (const float*) p;
            a[i]           = v[0];
            b[i]           = v[1];
            c[i]           = v[2];
            p += stride;
        }
    }

    bool cooking_build_model(const BuildDesc& in, Model*& out) {
        if (in.topo.node_count == 0) return false;
        Model* m = new(std::nothrow) Model();
        if (!m) return false;
        m->node_count = in.topo.node_count;
        if (in.topo.relations && in.topo.relation_count > 0) {
            auto& rv = in.topo.relations[0];
            if (rv.arity != 2) {
                delete m;
                return false;
            }
            m->edges.assign(rv.indices, rv.indices + rv.count * 2);
        }
        size_t npos      = 0, spos = 0;
        const void* ppos = find_field(in.state, "position", 3, npos, spos);
        if (!ppos || npos != m->node_count) {
            delete m;
            return false;
        }
        std::vector<float> x, y, z;
        load_vec3_aos(ppos, npos, spos, x, y, z);
        size_t ecount = m->edges.size() / 2;
        m->rest.resize(ecount);
        for (size_t e = 0; e < ecount; e++) {
            uint32_t a = m->edges[2 * e + 0];
            uint32_t b = m->edges[2 * e + 1];
            float dx   = x[b] - x[a];
            float dy   = y[b] - y[a];
            float dz   = z[b] - z[a];
            float l    = std::sqrt(dx * dx + dy * dy + dz * dz);
            m->rest[e] = l;
        }
        out = m;
        return true;
    }

    bool cooking_rebuild_model_from_commands(const Model& cur, const Command* cmds, size_t count, Model*& out, RemapPlan*& plan) {
        Model* m = new(std::nothrow) Model();
        if (!m) return false;
        m->node_count = cur.node_count;
        m->edges      = cur.edges;
        m->rest       = cur.rest;
        RemapPlan* rp = new(std::nothrow) RemapPlan();
        if (!rp) {
            delete m;
            return false;
        }
        rp->old_to_new.resize(m->node_count);
        for (uint32_t i = 0; i < m->node_count; i++) rp->old_to_new[i] = i;
        out  = m;
        plan = rp;
        return true;
    }
}

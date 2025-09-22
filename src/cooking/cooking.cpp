#include "cooking.h"
#include "core/common/utils.h"
#include "core/common/types.h"

#include <cstdint>
#include <cstring>
#include <new>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>

namespace sim {
    using namespace eng;

    static const void* find_field(const StateInit& st, const char* name, size_t comps, size_t& count, size_t& stride) {
        for (size_t i = 0; i < st.field_count; i++) {
            auto& f = st.fields[i];
            if (util::name_matches(name, f.name) && f.components == comps) {
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

    static void compute_islands_and_reorder(Model& m) {
        const uint32_t n = m.node_count;
        const std::size_t mcnt = m.edges.size() / 2;
        std::vector<std::vector<uint32_t>> adj(n);
        adj.reserve(n);
        for (std::size_t e = 0; e < mcnt; ++e) {
            uint32_t a = m.edges[2*e+0];
            uint32_t b = m.edges[2*e+1];
            if (a < n && b < n) { adj[a].push_back(b); adj[b].push_back(a); }
        }
        std::vector<int> comp(n, -1);
        int cc = 0;
        std::queue<uint32_t> q;
        for (uint32_t v = 0; v < n; ++v) {
            if (comp[v] != -1) continue;
            comp[v] = cc;
            q.push(v);
            while (!q.empty()) {
                auto u = q.front(); q.pop();
                for (auto w : adj[u]) {
                    if (comp[w] == -1) { comp[w] = cc; q.push(w); }
                }
            }
            cc++;
        }
        std::vector<std::vector<uint32_t>> edge_pairs(cc);
        std::vector<std::vector<float>>    edge_rest(cc);
        for (std::size_t e = 0; e < mcnt; ++e) {
            uint32_t a = m.edges[2*e+0];
            uint32_t b = m.edges[2*e+1];
            int cida = (a < n) ? comp[a] : -1;
            int cidb = (b < n) ? comp[b] : -1;
            int cid  = (cida == cidb && cida >= 0) ? cida : -1;
            if (cid < 0) cid = 0;
            edge_pairs[cid].push_back(a);
            edge_pairs[cid].push_back(b);
            if (e < m.rest.size()) edge_rest[cid].push_back(m.rest[e]);
            else edge_rest[cid].push_back(0.0f);
        }
        m.island_count = (uint32_t) cc;
        m.island_offsets.assign(cc + 1, 0);
        std::vector<uint32_t> new_edges; new_edges.reserve(m.edges.size());
        std::vector<float> new_rest; new_rest.reserve(m.rest.size());
        for (int cidx = 0; cidx < cc; ++cidx) {
            m.island_offsets[cidx] = (uint32_t) (new_rest.size());
            new_edges.insert(new_edges.end(), edge_pairs[cidx].begin(), edge_pairs[cidx].end());
            new_rest.insert(new_rest.end(), edge_rest[cidx].begin(), edge_rest[cidx].end());
        }
        m.island_offsets[cc] = (uint32_t) (new_rest.size());
        m.edges.swap(new_edges);
        m.rest.swap(new_rest);
    }

    static float dihedral_angle(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z,
                                uint32_t i0, uint32_t i1, uint32_t i2, uint32_t i3) {
        float e0x = x[i1]-x[i0], e0y = y[i1]-y[i0], e0z = z[i1]-z[i0];
        float e1x = x[i2]-x[i0], e1y = y[i2]-y[i0], e1z = z[i2]-z[i0];
        float e2x = x[i3]-x[i0], e2y = y[i3]-y[i0], e2z = z[i3]-z[i0];
        float n1x,n1y,n1z,n2x,n2y,n2z;
        util::cross3(e0x,e0y,e0z, e1x,e1y,e1z, n1x,n1y,n1z);
        util::cross3(e0x,e0y,e0z, e2x,e2y,e2z, n2x,n2y,n2z);
        float n1l = util::len3(n1x,n1y,n1z); float n2l = util::len3(n2x,n2y,n2z);
        if (n1l <= 1e-12f || n2l <= 1e-12f) return 0.0f;
        float c = util::dot3(n1x,n1y,n1z, n2x,n2y,n2z) / (n1l*n2l);
        c = std::clamp(c, -1.0f, 1.0f);
        float ang = std::acos(c);
        return ang;
    }

    bool cooking_build_model(const BuildDesc& in, Model*& out) {
        if (in.topo.node_count == 0) return false;
        Model* m = new(std::nothrow) Model();
        if (!m) return false;
        m->node_count = in.topo.node_count;
        if (in.topo.relations && in.topo.relation_count > 0) {
            auto& rv = in.topo.relations[0];
            if (rv.arity != 2) { delete m; return false; }
            m->edges.assign(rv.indices, rv.indices + rv.count * 2);
        }
        size_t npos = 0, spos = 0;
        const void* ppos = find_field(in.state, "position", 3, npos, spos);
        if (!ppos || npos != m->node_count) { delete m; return false; }
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
            m->rest[e] = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        for (size_t r = 0; r < in.topo.relation_count; ++r) {
            auto& rv = in.topo.relations[r];
            if (!rv.tag || std::strcmp(rv.tag, "bend_pairs") != 0) continue;
            if (rv.arity != 4) continue;
            m->bend_pairs.assign(rv.indices, rv.indices + rv.count * 4);
            m->bend_rest_angle.resize(rv.count);
            for (size_t i = 0; i < rv.count; ++i) {
                uint32_t i0 = m->bend_pairs[4*i+0];
                uint32_t i1 = m->bend_pairs[4*i+1];
                uint32_t i2 = m->bend_pairs[4*i+2];
                uint32_t i3 = m->bend_pairs[4*i+3];
                m->bend_rest_angle[i] = dihedral_angle(x,y,z,i0,i1,i2,i3);
            }
            break;
        }
        compute_islands_and_reorder(*m);
        m->node_remap.resize(m->node_count);
        for (uint32_t i = 0; i < m->node_count; ++i) m->node_remap[i] = i;
        if (in.pack.block_size > 0) m->layout_block_size = (uint32_t) in.pack.block_size;
        out = m;
        return true;
    }

    bool cooking_rebuild_model_from_commands(const Model& cur, const Command* cmds, size_t count, Model*& out, RemapPlan*& plan) {
        (void)cmds; (void)count;
        Model* m = new(std::nothrow) Model();
        if (!m) return false;
        m->node_count = cur.node_count;
        m->edges      = cur.edges;
        m->rest       = cur.rest;
        m->island_count   = cur.island_count;
        m->island_offsets = cur.island_offsets;
        m->node_remap      = cur.node_remap;
        m->layout_block_size = cur.layout_block_size;
        m->bend_pairs = cur.bend_pairs;
        m->bend_rest_angle = cur.bend_rest_angle;
        RemapPlan* rp = new(std::nothrow) RemapPlan();
        if (!rp) { delete m; return false; }
        rp->old_to_new.resize(m->node_count);
        for (uint32_t i = 0; i < m->node_count; i++) rp->old_to_new[i] = i;
        out  = m;
        plan = rp;
        return true;
    }
}

#include "api/build.h"
#include "core/model/model.h"
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <cstring>

namespace sim {
    static constexpr uint64_t HINACLOTH_CACHE_VERSION = 1ull;

    static uint64_t hash64(uint64_t x) {
        x ^= x >> 33;
        x *= 0xff51afd7ed558ccdull;
        x ^= x >> 33;
        x *= 0xc4ceb9fe1a85ec53ull;
        x ^= x >> 33;
        return x;
    }
    static uint64_t mix_ptr(const void* p) { return hash64((uint64_t) (uintptr_t) p); }
    static uint64_t mix_str(const char* s) {
        if (!s) return 0;
        uint64_t h = 1469598103934665603ull;
        while (*s) { h ^= (unsigned char)(*s++); h *= 1099511628211ull; }
        return h;
    }

    static uint64_t acc;

    static void hash_parameters(uint64_t& h, const Parameters& p) {
        for (size_t i = 0; i < p.count; ++i) {
            h ^= mix_str(p.items[i].name);
            h ^= hash64((uint64_t) p.items[i].type);
            const uint64_t* u = (const uint64_t*) &p.items[i].value;
            h ^= hash64(u[0]);
        }
    }

    static void hash_topology(uint64_t& h, const TopologyIn& t) {
        h ^= hash64((uint64_t) t.node_count);
        for (size_t i = 0; i < t.relation_count; ++i) {
            const RelationView& r = t.relations[i];
            h ^= mix_str(r.tag);
            h ^= hash64((uint64_t) r.arity);
            h ^= hash64((uint64_t) r.count);
            if (r.indices) {
                size_t n = r.count * r.arity;
                for (size_t k = 0; k < n; ++k) h ^= hash64((uint64_t) r.indices[k] + 0x9e3779b97f4a7c15ull * k);
            }
        }
    }

    static void hash_ops(uint64_t& h, const OperatorsDecl& o) {
        for (size_t i = 0; i < o.count; ++i) {
            const OperatorDecl& d = o.ops[i];
            h ^= mix_str(d.id);
            for (size_t k = 0; k < d.relation_tag_count; ++k) h ^= mix_str(d.relation_tags[k]);
            h ^= hash64((uint64_t) d.stage);
            for (size_t k = 0; k < d.field_count; ++k) {
                h ^= mix_str(d.fields[k].name);
                h ^= hash64((uint64_t) (d.fields[k].write ? 1 : 0));
            }
            h ^= hash64(d.enabled ? 1ull : 0ull);
        }
    }

    static void hash_policy(uint64_t& h, const Policy& p, const PackOptions& pack) {
        h ^= hash64((uint64_t) p.exec.layout);
        h ^= hash64((uint64_t) p.exec.backend);
        h ^= hash64((uint64_t) (p.exec.threads <= 0 ? 0 : p.exec.threads));
        h ^= hash64((uint64_t) (p.exec.deterministic ? 1 : 0));
        h ^= hash64((uint64_t) (p.exec.telemetry ? 1 : 0));
        h ^= hash64((uint64_t) p.solve.substeps);
        h ^= hash64((uint64_t) p.solve.iterations);
        h ^= hash64((uint64_t) (pack.block_size <= 0 ? 0 : pack.block_size));
    }

    static void hash_space(uint64_t& h, const SpaceDesc& s) {
        h ^= hash64((uint64_t) s.type);
        h ^= hash64((uint64_t) s.order);
        h ^= hash64((uint64_t) s.refinement_level);
    }

    void shell_cache_track_begin(const BuildDesc& d) {
        acc = 0;
        acc ^= hash64(HINACLOTH_CACHE_VERSION);
        hash_topology(acc, d.topo);
        hash_ops(acc, d.ops);
        hash_parameters(acc, d.params);
        hash_policy(acc, d.policy, d.pack);
        hash_space(acc, d.space);
        acc ^= hash64((uint64_t) d.validate);
    }

    void shell_cache_track_end() { (void) acc; }

    bool shell_cache_query(uint64_t& key_out) { key_out = acc; return acc != 0; }

    static Model* clone_model(const Model& m) {
        Model* c = new(std::nothrow) Model();
        if (!c) return nullptr;
        c->node_count = m.node_count;
        c->edges = m.edges;
        c->rest  = m.rest;
        c->island_count   = m.island_count;
        c->island_offsets = m.island_offsets;
        c->node_remap = m.node_remap;
        c->layout_block_size = m.layout_block_size;
        c->bend_pairs = m.bend_pairs;
        c->bend_rest_angle = m.bend_rest_angle;
        return c;
    }

    struct ModelCacheEntry { std::vector<uint32_t> edges; std::vector<float> rest; uint32_t node_count; std::vector<uint32_t> island_offsets; uint32_t island_count; std::vector<uint32_t> node_remap; uint32_t layout_block_size; std::vector<uint32_t> bend_pairs; std::vector<float> bend_rest_angle; };
    static std::unordered_map<uint64_t, ModelCacheEntry> g_cache;

    bool shell_cache_load(uint64_t key, Model*& out) {
        auto it = g_cache.find(key);
        if (it == g_cache.end()) return false;
        const ModelCacheEntry& e = it->second;
        Model* m = new(std::nothrow) Model();
        if (!m) return false;
        m->node_count = e.node_count;
        m->edges = e.edges;
        m->rest  = e.rest;
        m->island_count = e.island_count;
        m->island_offsets = e.island_offsets;
        m->node_remap = e.node_remap;
        m->layout_block_size = e.layout_block_size;
        m->bend_pairs = e.bend_pairs;
        m->bend_rest_angle = e.bend_rest_angle;
        out = m;
        return true;
    }

    void shell_cache_store(uint64_t key, const Model& m) {
        ModelCacheEntry e;
        e.node_count = m.node_count;
        e.edges = m.edges;
        e.rest  = m.rest;
        e.island_count = m.island_count;
        e.island_offsets = m.island_offsets;
        e.node_remap = m.node_remap;
        e.layout_block_size = m.layout_block_size;
        e.bend_pairs = m.bend_pairs;
        e.bend_rest_angle = m.bend_rest_angle;
        g_cache[key] = std::move(e);
    }
}

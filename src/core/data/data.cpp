#include "data.h"
#include "core/model/model.h"
#include "remap.h"
#include "api/build.h"
#include "api/commands.h"
#include "api/parameters_in.h"

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

    bool core_data_create_from_state(const BuildDesc& in, const Model& m, Data*& out) {
        Data* d = new(std::nothrow) Data();
        if (!d) return false;
        d->gx = 0.0f;
        d->gy = -9.8f;
        d->gz = 0.0f;
        for (size_t i = 0; i < in.params.count; i++) {
            auto& p = in.params.items[i];
            if (p.type == ParamType::F32) {
                if (std::strcmp(p.name, "gravity_x") == 0) d->gx = p.value.f32;
                if (std::strcmp(p.name, "gravity_y") == 0) d->gy = p.value.f32;
                if (std::strcmp(p.name, "gravity_z") == 0) d->gz = p.value.f32;
            }
        }
        size_t npos      = 0, nvel = 0;
        size_t spos      = 0, svel = 0;
        const void* ppos = find_field(in.state, "position", 3, npos, spos);
        const void* pvel = find_field(in.state, "velocity", 3, nvel, svel);
        if (!ppos || npos != m.node_count) {
            delete d;
            return false;
        }
        load_vec3_aos(ppos, npos, spos, d->x, d->y, d->z);
        d->vx.assign(npos, 0.0f);
        d->vy.assign(npos, 0.0f);
        d->vz.assign(npos, 0.0f);
        if (pvel) {
            if (nvel != m.node_count) {
                delete d;
                return false;
            }
            load_vec3_aos(pvel, nvel, svel, d->vx, d->vy, d->vz);
        }
        d->px = d->x;
        d->py = d->y;
        d->pz = d->z;
        out   = d;
        return true;
    }

    bool core_data_apply_overrides(Data& d, const Command* cmds, size_t count) {
        for (size_t i = 0; i < count; i++) {
            auto& c = cmds[i];
            if (c.tag == CommandTag::SetParam && c.data && c.bytes > 0) {
                const char* name = (const char*) c.data;
                if (std::strcmp(name, "gravity_x") == 0) {
                    if (c.bytes >= sizeof(const char*) + sizeof(float)) {
                        const float* v = (const float*) ((const char*) c.data + sizeof(const char*));
                        d.gx           = *v;
                    }
                }
                if (std::strcmp(name, "gravity_y") == 0) {
                    if (c.bytes >= sizeof(const char*) + sizeof(float)) {
                        const float* v = (const float*) ((const char*) c.data + sizeof(const char*));
                        d.gy           = *v;
                    }
                }
                if (std::strcmp(name, "gravity_z") == 0) {
                    if (c.bytes >= sizeof(const char*) + sizeof(float)) {
                        const float* v = (const float*) ((const char*) c.data + sizeof(const char*));
                        d.gz           = *v;
                    }
                }
            }
        }
        return true;
    }

    bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd) {
        Data* d = new(std::nothrow) Data();
        if (!d) return false;
        size_t n = oldd.x.size();
        if (plan.old_to_new.size() == n) {
            d->x.resize(n);
            d->y.resize(n);
            d->z.resize(n);
            d->vx.resize(n);
            d->vy.resize(n);
            d->vz.resize(n);
            d->px.resize(n);
            d->py.resize(n);
            d->pz.resize(n);
            for (size_t i = 0; i < n; i++) {
                uint32_t j = plan.old_to_new[i];
                if (j < n) {
                    d->x[j]  = oldd.x[i];
                    d->y[j]  = oldd.y[i];
                    d->z[j]  = oldd.z[i];
                    d->vx[j] = oldd.vx[i];
                    d->vy[j] = oldd.vy[i];
                    d->vz[j] = oldd.vz[i];
                    d->px[j] = oldd.px[i];
                    d->py[j] = oldd.py[i];
                    d->pz[j] = oldd.pz[i];
                }
            }
        } else {
            d->x  = oldd.x;
            d->y  = oldd.y;
            d->z  = oldd.z;
            d->vx = oldd.vx;
            d->vy = oldd.vy;
            d->vz = oldd.vz;
            d->px = oldd.px;
            d->py = oldd.py;
            d->pz = oldd.pz;
        }
        d->gx = oldd.gx;
        d->gy = oldd.gy;
        d->gz = oldd.gz;
        newd  = d;
        return true;
    }

    void core_data_destroy(Data* d) {
        delete d;
    }
}

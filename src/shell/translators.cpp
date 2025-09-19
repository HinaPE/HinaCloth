#include "api/build.h"

namespace sim {
    static void normalize_units(BuildDesc& d) {
    }

    static void normalize_fields(BuildDesc& d) {
    }

    static void normalize_policy(BuildDesc& d) {
        if (d.policy.exec.threads == 0) d.policy.exec.threads = -1;
    }

    void shell_translate(BuildDesc& d) {
        normalize_units(d);
        normalize_fields(d);
        normalize_policy(d);
    }
}

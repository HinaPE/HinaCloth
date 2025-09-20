#include "api/build.h"

namespace sim {
    static void normalize_units([[maybe_unused]] BuildDesc& d) {
        // Placeholder: if units were provided, convert to internal SI.
    }

    static void normalize_fields([[maybe_unused]] BuildDesc& d) {
        // No in-place renaming because StateInit.fields points to const FieldView array from user.
        // Alias handling is implemented in validators/cooking/data find_field functions.
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

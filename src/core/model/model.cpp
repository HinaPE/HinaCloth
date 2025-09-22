#include "model.h"

namespace sim {
    void core_model_destroy(Model* m) noexcept {
        delete m;
    }
}

#include "model.h"

namespace sim {
    void core_model_destroy(Model* m) {
        delete m;
    }
}

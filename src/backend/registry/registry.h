#ifndef HINACLOTH_REGISTRY_H
#define HINACLOTH_REGISTRY_H
#include "api/policy_in.h"
#include "api/sim.h"
#include "core/model/model.h"

namespace sim {
    bool backends_choose(const Model& m, const PolicyExec& exec, Chosen& out);
}
#endif //HINACLOTH_REGISTRY_H

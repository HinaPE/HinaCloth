/*
 * File: build.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_BUILD_H
#define HINACLOTH_BUILD_H
#include "state_in.h"
#include "parameters_in.h"
#include "topology_in.h"
#include "policy_in.h"
#include "space_in.h"
#include "operators_in.h"
#include "events_in.h"

namespace sim {
    enum class ValidateLevel { Strict, Tolerant };

    struct PackOptions {
        bool lazy_pack;
        int block_size;
    };

    struct BuildDesc {
        StateInit state;
        Parameters params;
        TopologyIn topo;
        Policy policy;
        SpaceDesc space;
        OperatorsDecl ops;
        EventsScript events;
        ValidateLevel validate;
        PackOptions pack;
    };
}
#endif

/*
 * File: registry.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_REGISTRY_H
#define HINACLOTH_REGISTRY_H
#include "core/common/types.h"
#include "core/model/model.h"

namespace sim { namespace eng {
    bool backends_choose(const ::sim::Model& m, const PolicyExec& exec, Chosen& out);
}}
#endif

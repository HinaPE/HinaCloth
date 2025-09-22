/*
 * File: topology_in.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_TOPOLOGY_IN_H
#define HINACLOTH_TOPOLOGY_IN_H
#include <cstdint>

namespace sim
{
    struct RelationView
    {
        const uint32_t* indices;
        size_t arity;
        size_t count;
        const char* tag;
    };

    struct TopologyIn
    {
        uint32_t node_count;
        const RelationView* relations;
        size_t relation_count;
    };
}
#endif

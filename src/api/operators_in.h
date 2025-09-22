/*
 * File: operators_in.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_OPERATORS_IN_H
#define HINACLOTH_OPERATORS_IN_H
#include <cstddef>

namespace sim
{
    enum class OpStage { PreSolve, Solve, PostSolve };

    struct FieldUse
    {
        const char* name;
        bool write;
    };

    struct OperatorDecl
    {
        const char* id;
        const char* const* relation_tags;
        size_t relation_tag_count;
        const FieldUse* fields;
        size_t field_count;
        OpStage stage;
        bool enabled;
    };

    struct OperatorsDecl
    {
        const OperatorDecl* ops;
        size_t count;
    };
}

#endif

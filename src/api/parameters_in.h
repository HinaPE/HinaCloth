#ifndef HINACLOTH_PARAMETERS_IN_H
#define HINACLOTH_PARAMETERS_IN_H
#include <cstdint>

namespace sim
{
    enum class ParamType
    {
        F32,
        I32,
        U32,
        BOOL,
        VEC2,
        VEC3,
        VEC4,
        MAT3,
        MAT4,
        STRING,
        BLOB
    };

    struct ParamVec2
    {
        float x;
        float y;
    };

    struct ParamVec3
    {
        float x;
        float y;
        float z;
    };

    struct ParamVec4
    {
        float x;
        float y;
        float z;
        float w;
    };

    struct ParamMat3
    {
        float m[9];
    };

    struct ParamMat4
    {
        float m[16];
    };

    struct ParamBlob
    {
        const void* data;
        size_t bytes;
    };

    union ParamValue
    {
        float f32;
        int32_t i32;
        uint32_t u32;
        int32_t b;
        ParamVec2 vec2;
        ParamVec3 vec3;
        ParamVec4 vec4;
        ParamMat3 mat3;
        ParamMat4 mat4;
        const char* str;
        ParamBlob blob;
    };

    struct Param
    {
        const char* name;
        ParamType type;
        ParamValue value;
    };

    struct Parameters
    {
        const Param* items;
        size_t count;
    };
}
#endif //HINACLOTH_PARAMETERS_IN_H
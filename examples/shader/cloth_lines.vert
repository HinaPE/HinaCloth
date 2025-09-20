#version 460
layout(location=0) in vec3 inPos; // simulation position (x,y,z)
layout(push_constant) uniform PC {
    // Maps XZ plane into NDC XY: p = inPos.xz * scale + offset
    vec2 scale;
    vec2 offset;
} pc;
void main()
{
    vec2 p = inPos.xz * pc.scale + pc.offset;
    gl_Position = vec4(p, 0.0, 1.0);
}


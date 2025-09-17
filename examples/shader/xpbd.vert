// xpbd.vert
#version 450
layout(location=0) in vec2 inPos;

layout(push_constant) uniform Push {
    vec2 scale;   // NDC scale
    vec2 offset;  // NDC offset
    float pointSize; float _pad0; float _pad1; float _pad2;
    vec4 color;
} pc;

layout(location=0) out vec4 vColor;

void main() {
    vec2 ndc = inPos * pc.scale + pc.offset;
    gl_Position = vec4(ndc, 0.0, 1.0);
    gl_PointSize = pc.pointSize;
    vColor = pc.color;
}


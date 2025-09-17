#version 450

layout(location = 0) in vec2 inPos;

layout(push_constant) uniform Push {
    vec2 a; // scale to NDC
    vec2 b; // translate to NDC
    vec4 color;
    float pointSize;
} pc;

layout(location = 0) out vec4 vColor;

void main() {
    vec2 ndc = vec2(pc.a.x * inPos.x + pc.b.x,
                    pc.a.y * inPos.y + pc.b.y);
    gl_Position = vec4(ndc, 0.0, 1.0);
    gl_PointSize = pc.pointSize;
    vColor = pc.color;
}


#version 460 core

layout(push_constant) uniform PushConstants {
    mat4 mvp;
    float pointSize;
    float lineWidth;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 vColor;

void main() {
    vColor = inColor;
    gl_Position = pc.mvp * vec4(inPosition, 1.0);
    gl_PointSize = pc.pointSize;
}

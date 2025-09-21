#version 460
layout(location=0) in vec3 inPos;
layout(location=0) out vec4 vColor;
layout(push_constant) uniform PC { mat4 mvp; vec4 color; float pointSize; } pc;
void main(){
    gl_Position = pc.mvp * vec4(inPos, 1.0);
    vColor = pc.color;
    gl_PointSize = pc.pointSize;
}


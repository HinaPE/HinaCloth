#version 460
layout(location=0) in vec3 inPos;
layout(location=1) in vec3 inNormal;
layout(location=0) out vec3 vNormal;
layout(push_constant) uniform PC { mat4 mvp; vec4 color; float pointSize; } pc;
void main(){
    gl_Position = pc.mvp * vec4(inPos, 1.0);
    vNormal = normalize(inNormal);
}


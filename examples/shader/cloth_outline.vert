#version 460
layout(location=0) in vec3 inPos;
layout(location=1) in vec3 inNormal;
layout(push_constant) uniform PC { mat4 mvp; mat4 lightMVP; vec4 color; float pointSize; float outline; float shadowStrength; float shadowTexel; } pc;
void main(){
    vec3 p = inPos + normalize(inNormal) * pc.outline;
    gl_Position = pc.mvp * vec4(p, 1.0);
}


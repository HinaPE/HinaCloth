#version 460
layout(location=0) in vec3 inPos;
layout(push_constant) uniform PC { mat4 mvp; mat4 lightMVP; vec4 color; float pointSize; float outline; float shadowStrength; float shadowTexel; } pc;
void main(){ gl_Position = pc.lightMVP * vec4(inPos, 1.0); }


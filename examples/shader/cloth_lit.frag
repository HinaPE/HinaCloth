#version 460
layout(location=0) in vec3 vNormal;
layout(location=0) out vec4 outColor;
layout(push_constant) uniform PC { mat4 mvp; vec4 color; float pointSize; } pc;
void main(){
    vec3 N = normalize(vNormal);
    vec3 L = normalize(vec3(0.4, 0.7, 0.5));
    float diff = max(dot(N, L), 0.0);
    float ambient = 0.25;
    vec3 base = pc.color.rgb;
    vec3 c = base * (ambient + 0.75*diff);
    outColor = vec4(c, pc.color.a);
}


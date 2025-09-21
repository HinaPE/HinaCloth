#version 460
layout(location=0) in vec3 vNormal;
layout(location=1) in vec4 vLightPos;
layout(location=0) out vec4 outColor;
layout(push_constant) uniform PC { mat4 mvp; mat4 lightMVP; vec4 color; float pointSize; float outline; float shadowStrength; float shadowTexel; } pc;
layout(set=0,binding=0) uniform sampler2D shadowMap;
void main(){
    vec3 N = normalize(vNormal);
    vec3 L = normalize(vec3(0.4, 0.7, 0.5));
    float NdL = max(dot(N, L), 0.0);
    vec3 base = pc.color.rgb;
    vec3 proj = vLightPos.xyz / max(vLightPos.w, 1e-6);
    vec2 uv = proj.xy * 0.5 + 0.5;
    float depth = proj.z * 0.5 + 0.5;
    float visibility = 1.0;
    if (uv.x>=0.0 && uv.x<=1.0 && uv.y>=0.0 && uv.y<=1.0) {
        float bias = max(0.0005, 0.001 * (1.0 - NdL));
        float sum = 0.0;
        for (int dy=-1; dy<=1; ++dy){
            for (int dx=-1; dx<=1; ++dx){
                vec2 offs = vec2(dx, dy) * pc.shadowTexel;
                float sdepth = texture(shadowMap, uv + offs).r;
                sum += (depth - bias > sdepth) ? 0.0 : 1.0;
            }
        }
        visibility = sum / 9.0;
    }
    float ambient = 0.25;
    float lighting = ambient + NdL * mix(1.0, visibility, pc.shadowStrength);
    outColor = vec4(base * lighting, pc.color.a);
}


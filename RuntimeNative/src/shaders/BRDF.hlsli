#ifndef BRDF_HLSLI
#define BRDF_HLSLI


static const float PI = 3.14159265f;


float3 FresnelSchlick(const float cosTheta, const float3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}


float TrowbridgeReitz(const float3 N, const float3 H, const float roughness) {
    const float a = roughness * roughness;
    const float a2 = a * a;
    const float NdotH = max(dot(N, H), 0.0);
    const float NdotH2 = NdotH * NdotH;
	
    const float num = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
	
    return num / denom;
}


float SchlickTrowbridgeReitz(const float NdotV, const float roughness) {
    const float r = (roughness + 1.0);
    const float k = (r * r) / 8.0;

    const float num = NdotV;
    const float denom = NdotV * (1.0 - k) + k;
	
    return num / denom;
}


float Smith(const float3 N, const float3 V, const float3 L, const float roughness) {
    const float NdotV = max(dot(N, V), 0.0);
    const float NdotL = max(dot(N, L), 0.0);
    const float ggx2 = SchlickTrowbridgeReitz(NdotV, roughness);
    const float ggx1 = SchlickTrowbridgeReitz(NdotL, roughness);
	
    return ggx1 * ggx2;
}


float3 CookTorrance(const float3 N, const float3 V, const float3 L, const float3 albedo, const float metallic, const float roughness, const float3 lightColor, const float lightIntensity) {
    float3 F0 = float3(0.04, 0.04, 0.04);
    F0 = lerp(F0, albedo, metallic);
	           
    // reflectance equation
    float3 Lo = float3(0.0, 0.0, 0.0);
        // calculate per-light radiance
    const float3 H = normalize(V + L);
    const float3 radiance = lightColor * lightIntensity;
        
        // cook-torrance brdf
    const float NDF = TrowbridgeReitz(N, H, roughness);
    const float G = Smith(N, V, L, roughness);
    const float3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);
        
    const float3 kS = F;
    float3 kD = float3(1.0, 1.0, 1.0) - kS;
    kD *= 1.0 - metallic;
        
    const float3 numerator = NDF * G * F;
    const float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
    const float3 specular = numerator / denominator;
            
        // add to outgoing radiance Lo
    const float NdotL = max(dot(N, L), 0.0);
    Lo += (kD * albedo / PI + specular) * radiance * NdotL;
   
    return Lo;
}

#endif
#include "MeshVSOut.hlsli"
#include "ShaderInterop.h"
#include "BRDF.hlsli"
#include "ShadowFilteringModes.h"
#include "ShadowSampling.hlsli"


TEXTURE2D(gAlbedoMap, float4, RES_SLOT_ALBEDO_MAP);
TEXTURE2D(gMetallicMap, float, RES_SLOT_METALLIC_MAP);
TEXTURE2D(gRoughnessMap, float, RES_SLOT_ROUGHNESS_MAP);
TEXTURE2D(gAoMap, float, RES_SLOT_AO_MAP);
TEXTURE2D(gNormalMap, float3, RES_SLOT_NORMAL_MAP);
TEXTURE2D(gPunctualShadowAtlas, float, RES_SLOT_PUNCTUAL_SHADOW_ATLAS);
TEXTURE2D(gDirShadowAtlas, float, RES_SLOT_DIR_SHADOW_ATLAS);

STRUCTUREDBUFFER(gLights, ShaderLight, RES_SLOT_LIGHTS);


int CalculateShadowCascadeIdx(const float fragViewPosZ) {
    return dot(gPerCamConstants.shadowCascadeSplitDistances < fragViewPosZ, 1.0);
}


float3 VisualizeShadowCascades(const float fragViewPosZ) {
    const int cascadeIdx = CalculateShadowCascadeIdx(fragViewPosZ);

    if (cascadeIdx >= gPerFrameConstants.shadowCascadeCount) {
        return float3(1, 1, 1);
    }

    float3 ret;

    switch (cascadeIdx) {
    case 0:
        ret = float3(108, 110, 160);
        break;
    case 1:
        ret = float3(184, 216, 186);
        break;
    case 2:
        ret = float3(252, 221, 188);
        break;
    case 3:
        ret = float3(239, 149, 157);
        break;
    default:
        ret = float3(1, 1, 1); // This should never be reached
        break;
    }

    return pow(ret / 255.0, 2.2);
}


inline float SampleShadowCascadeFromAtlas(const Texture2D<float> atlas, const float3 fragWorldPos, const uint lightIdx, const uint shadowMapIdx, const float3 fragNormal) {
    uint atlasSize;
    atlas.GetDimensions(atlasSize, atlasSize);
    const float atlasTexelSize = 1.0 / atlasSize;
    const float shadowMapTexelSize = atlasTexelSize / gLights[lightIdx].shadowAtlasCellSizes[shadowMapIdx];

    const float4 posLClip = mul(float4(fragWorldPos + fragNormal * shadowMapTexelSize * gLights[lightIdx].normalBias, 1), gLights[lightIdx].shadowViewProjMatrices[shadowMapIdx]);
    float3 posLNdc = posLClip.xyz / posLClip.w;
    posLNdc.xy = posLNdc.xy * float2(0.5, -0.5) + 0.5;

    const float2 uv = posLNdc.xy * gLights[lightIdx].shadowAtlasCellSizes[shadowMapIdx] + gLights[lightIdx].shadowAtlasCellOffsets[shadowMapIdx];
    const float depth = posLNdc.z + shadowMapTexelSize * gLights[lightIdx].depthBias;

    switch (gPerFrameConstants.shadowFilteringMode) {
    case SHADOW_FILTERING_NONE:
        return SampleShadowMapNoFilter(atlas, uv, depth);
    case SHADOW_FILTERING_HARDWARE_PCF:
        return SampleShadowMapHardwarePCF(atlas, uv, depth);
    case SHADOW_FILTERING_PCF_3x3:
        return SampleShadowMapPCF3x34TapFast(atlas, uv, depth);
    case SHADOW_FILTERING_PCF_TENT_3x3:
        return SampleShadowMapPCF3x3Tent4Tap(atlas, uv, depth);
    case SHADOW_FILTERING_PCF_TENT_5x5:
        return SampleShadowMapPCF5x5Tent9Tap(atlas, uv, depth);
    default:
        return 1.0;
    }
}


inline float CalculateAttenuation(const float distance) {
    return 1 / pow(distance, 2);
}


inline float3 CalculateDirLight(const float3 N, const float3 V, const float3 albedo, const float metallic, const float roughness, const int lightIdx, const float3 fragPosWorld, const float fragViewPosZ) {
    const float3 L = -gLights[lightIdx].direction;

    float3 lighting;

    [branch]
    if (gPerFrameConstants.visualizeShadowCascades) {
        lighting = VisualizeShadowCascades(fragViewPosZ);
    }
    else {
        lighting = CookTorrance(N, V, L, albedo, metallic, roughness, gLights[lightIdx].color, gLights[lightIdx].intensity, 1);
    }


    [branch]
    if (gLights[lightIdx].isCastingShadow) {
        const int cascadeIdx = CalculateShadowCascadeIdx(fragViewPosZ);

        [branch]
        if (cascadeIdx < gPerFrameConstants.shadowCascadeCount) {
            lighting *= SampleShadowCascadeFromAtlas(gDirShadowAtlas, fragPosWorld, lightIdx, cascadeIdx, N);
        }
    }

    return lighting;
}


inline float3 CalculateSpotLight(const float3 N, const float3 V, const float3 albedo, const float metallic, const float roughness, const int lightIdx, const float3 fragWorldPos) {
    float3 L = gLights[lightIdx].position - fragWorldPos;
    const float dist = length(L);
    L = normalize(L);

    const float rangeMul = float(dist <= gLights[lightIdx].range);
    const float thetaCos = dot(L, -gLights[lightIdx].direction);
    const float eps = gLights[lightIdx].halfInnerAngleCos - gLights[lightIdx].halfOuterAngleCos;
    const float intensity = saturate((thetaCos - gLights[lightIdx].halfOuterAngleCos) / eps);

    float3 lighting = CookTorrance(N, V, L, albedo, metallic, roughness, gLights[lightIdx].color, gLights[lightIdx].intensity, CalculateAttenuation(dist));
    lighting *= intensity;
    lighting *= rangeMul;

    [branch]
    if (gLights[lightIdx].isCastingShadow) {
        lighting *= SampleShadowCascadeFromAtlas(gPunctualShadowAtlas, fragWorldPos, lightIdx, 0, N);
    }
    
    return lighting;
}


inline float3 CalculatePointLight(const float3 N, const float3 V, const float3 albedo, const float metallic, const float roughness, const int lightIdx, const float3 fragWorldPos) {
    float3 L = gLights[lightIdx].position - fragWorldPos;
    const float dist = length(L);
    L = normalize(L);

    const float rangeMul = float(dist <= gLights[lightIdx].range);

    float3 lighting = CookTorrance(N, V, L, albedo, metallic, roughness, gLights[lightIdx].color, gLights[lightIdx].intensity, CalculateAttenuation(dist));
    lighting *= rangeMul;

    [branch]
    if (gLights[lightIdx].isCastingShadow) {
        const float3 dirToFrag = fragWorldPos - gLights[lightIdx].position;

        uint maxIdx = abs(dirToFrag.x) > abs(dirToFrag.y) ? 0 : 1;
        maxIdx = abs(dirToFrag[maxIdx]) > abs(dirToFrag.z) ? maxIdx : 2;
        uint shadowMapIdx = maxIdx * 2;

        if (sign(dirToFrag[maxIdx]) < 0) {
            shadowMapIdx += 1;
        }

        [branch]
        if (gLights[lightIdx].sampleShadowMap[shadowMapIdx]) {
            lighting *= SampleShadowCascadeFromAtlas(gPunctualShadowAtlas, fragWorldPos, lightIdx, shadowMapIdx, N);
        }
    }

    return lighting;
}


float4 main(const MeshVsOut vsOut) : SV_TARGET {
    float3 albedo = material.albedo;

    if (material.sampleAlbedo) {
        albedo *= pow(gAlbedoMap.Sample(gSamplerAf16, vsOut.uv).rgb, 2.2);
    }

    float metallic = material.metallic;

    if (material.sampleMetallic) {
        metallic *= gMetallicMap.Sample(gSamplerAf16, vsOut.uv).r;
    }

    float roughness = material.roughness;

    if (material.sampleRoughness) {
        roughness *= gRoughnessMap.Sample(gSamplerAf16, vsOut.uv).r;
    }

    float ao = material.ao;

    if (material.sampleAo) {
        ao *= gAoMap.Sample(gSamplerAf16, vsOut.uv).r;
    }

    float3 N = normalize(vsOut.normal);

    if (material.sampleNormal) {
        N = gNormalMap.Sample(gSamplerAf16, vsOut.uv).rgb;
        N *= 2.0;
        N -= 1.0;
        N = normalize(mul(N, vsOut.tbnMtx));
    }

    const float3 V = normalize(gPerCamConstants.camPos - vsOut.worldPos);

    float3 outColor = 0.03 * albedo * ao;

    uint lightCount;
    uint _;
    gLights.GetDimensions(lightCount, _);

    for (uint i = 0; i < lightCount; i++) {
        switch (gLights[i].type) {
        case 0:{
                outColor += CalculateDirLight(N, V, albedo, metallic, roughness, i, vsOut.worldPos, vsOut.viewPosZ);
                break;
            }
        case 1:{
                outColor += CalculateSpotLight(N, V, albedo, metallic, roughness, i, vsOut.worldPos);
                break;
            }
        case 2:{
                outColor += CalculatePointLight(N, V, albedo, metallic, roughness, i, vsOut.worldPos);
                break;
            }
        default:{
                break;
            }
        }
    }

    return float4(outColor, 1);
}
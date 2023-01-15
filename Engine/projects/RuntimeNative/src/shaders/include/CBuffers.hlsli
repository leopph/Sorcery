#ifndef CBUFFERS_HLSLI
#define CBUFFERS_HLSLI

#include "Lights.hlsli"
#include "PbrMaterial.hlsli"

cbuffer PerFrameConstants : register(b0) {
    DirectionalLight dirLight;
    bool calcDirLight;
    float invGamma;
}

cbuffer PerCameraConstants : register(b1) {
    row_major float4x4 viewProjMat;
    float3 camPos;
};

cbuffer PbrMaterialBuffer : register(b2) {
    PbrMaterial material;
};

cbuffer PerModelConstants : register(b3) {
    row_major float4x4 modelMat;
    row_major float3x3 normalMat;
};

#endif
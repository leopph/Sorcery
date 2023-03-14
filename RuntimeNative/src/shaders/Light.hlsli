#ifndef LIGHTS_HLSLI
#define LIGHTS_HLSLI

struct Light {
    float3 color;
    float intensity;

    float3 direction;
    int type;

    float shadowNearPlane;
    float range;
    float innerAngleCos;
    int isCastingShadow;

    float3 position;
    float outerAngleCos;
};

#endif
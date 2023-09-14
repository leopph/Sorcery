#include "ShaderInterop.h"
#include "DepthNormalVsOut.hlsli"


struct VsIn {
  float3 positionOS : POSITION;
  float3 normalOS : NORMAL;
  float2 uv : TEXCOORD;
  float3 tangentOS : TANGENT;
};


DepthNormalVsOut main(const VsIn vsIn) {
  const float4 positionWS = mul(float4(vsIn.positionOS, 1), gPerDrawConstants.modelMtx);
  const float4 positionCS = mul(positionWS, gPerViewConstants.viewProjMtx);

  const float3 normalWS = normalize(mul(vsIn.normalOS, (float3x3)gPerDrawConstants.invTranspModelMtx));
  float3 tangentWS = normalize(mul(vsIn.tangentOS, (float3x3)gPerDrawConstants.modelMtx));
  tangentWS = normalize(tangentWS - dot(tangentWS, normalWS) * normalWS);
  const float3 bitangentWS = cross(normalWS, tangentWS);
  const float3x3 tbnMtxWS = float3x3(tangentWS, bitangentWS, normalWS);

  DepthNormalVsOut ret;
  ret.positionCS = positionCS;
  ret.normalWS = normalWS;
  ret.uv = vsIn.uv;
  ret.tbnMtxWS = tbnMtxWS;
  return ret;
}

#ifndef IRRADIANCE_HLSLI
#define IRRADIANCE_HLSLI

#include "common.hlsli"
#include "shader_interop.h"

DECLARE_PARAMS(IrradianceDrawParams);
DECLARE_DRAW_CALL_PARAMS(g_draw_call_params);


float const static kPi = 3.14159265f;


struct VertexOut {
  float4 pos_cs : SV_Position;
  float4 pos_ws : POSITIONWS;
  uint rt_array_idx : SV_RenderTargetArrayIndex;
};


float3 ImportanceSampleGGX(float2 Xi, float Roughness, float3 N) {
  float a = Roughness * Roughness;
  float Phi = 2 * kPi * Xi.x;
  float CosTheta = sqrt((1 - Xi.y) / (1 + (a * a - 1) * Xi.y));
  float SinTheta = sqrt(1 - CosTheta * CosTheta);
  float3 H;
  H.x = SinTheta * cos(Phi);
  H.y = SinTheta * sin(Phi);
  H.z = CosTheta;
  float3 UpVector = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
  float3 TangentX = normalize(cross(UpVector, N));
  float3 TangentY = cross(N, TangentX);
  // Tangent to world space
  return TangentX * H.x + TangentY * H.y + N * H.z;
}


VertexOut VsMain(uint vertex_id : SV_VertexID) {
  vertex_id += g_draw_call_params.base_vertex;

  StructuredBuffer<float4> const positions = ResourceDescriptorHeap[g_params.pos_buf_idx];
  float4 const pos_os = positions[vertex_id];

  float4 const pos_cs = mul(pos_os, g_params.view_proj_mtx);

  VertexOut ret;
  ret.pos_cs = pos_cs;
  ret.pos_ws = pos_os;
  ret.rt_array_idx = g_params.rt_array_idx;
  return ret;
}


float4 PsMain(VertexOut vertex_out) : SV_Target {
  float3 const normal = normalize(vertex_out.pos_ws.xyz);
  float3 const right = normalize(cross(float3(0, 1, 0), normal));
  float3 const up = normalize(cross(normal, right));

  TextureCube const cubemap = ResourceDescriptorHeap[g_params.cubemap_idx];
  SamplerState const samp = SamplerDescriptorHeap[g_params.samp_idx];

  float3 irradiance = float3(0, 0, 0);

  float const sample_delta = 0.025;
  int sample_count = 0;

  for (float phi = 0.0; phi < 2.0 * kPi; phi += sample_delta) {
    for (float theta = 0.0; theta < 0.5 * kPi; theta += sample_delta) {
      // spherical to cartesian (in tangent space)
      float3 tangent_sample = float3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
      // tangent space to world
      float3 sample_vec = tangent_sample.x * right + tangent_sample.y * up + tangent_sample.z * normal;

      irradiance += cubemap.Sample(samp, sample_vec).rgb * cos(theta) * sin(theta);
      sample_count++;
    }
  }

  return float4(irradiance * kPi / float(sample_count), 1);
}

#endif

#ifndef SHADER_INTEROP_H
#define SHADER_INTEROP_H


#ifdef __cplusplus

#include "../../Math.hpp"


namespace sorcery {
using float2 = Vector2;
using float3 = Vector3;
using float4 = Vector4;

using float4x4 = Matrix4;
using uint = unsigned;
using BOOL = int;

#define row_major
#else
typedef bool BOOL;
#define MAKE_RES_SLOT(slot) t##slot
#define MAKE_SAMPLER_SLOT(slot) s##slot
#define MAKE_UAV_SLOT(slot) u##slot

#define TEXTURE2D(name, type, slot) Texture2D<type> name : register(MAKE_RES_SLOT(slot))
#define TEXTURE2DMS(name, type, slot) Texture2DMS<type> name : register(MAKE_RES_SLOT(slot))
#define TEXTURE2DARRAY(name, type, slot) Texture2DArray<type> name : register(MAKE_RES_SLOT(slot))
#define TEXTURECUBE(name, type, slot) TextureCube<type> name : register(MAKE_RES_SLOT(slot))
#define SAMPLERCOMPARISONSTATE(name, slot) SamplerComparisonState name : register(MAKE_SAMPLER_SLOT(slot))
#define SAMPLERSTATE(name, slot) SamplerState name : register(MAKE_SAMPLER_SLOT(slot))
#define STRUCTUREDBUFFER(name, type, slot) StructuredBuffer<type> name : register(MAKE_RES_SLOT(slot))
#define RWTEXTURE2D(name, type, slot) RWTexture2D<type> name : register(MAKE_UAV_SLOT(slot))
#endif

#define INVALID_RES_IDX ((uint)-1)

#define RES_SLOT_ALBEDO_MAP 0
#define RES_SLOT_METALLIC_MAP 1
#define RES_SLOT_ROUGHNESS_MAP 2
#define RES_SLOT_AO_MAP 3
#define RES_SLOT_NORMAL_MAP 4
#define RES_SLOT_OPACITY_MASK 5
#define RES_SLOT_PUNCTUAL_SHADOW_ATLAS 6
#define RES_SLOT_DIR_SHADOW_ATLAS 7
#define RES_SLOT_DIR_SHADOW_MAP_ARRAY 7
#define RES_SLOT_LIGHTS 8
#define RES_SLOT_SSAO_TEX 9
#define RES_SLOT_POST_PROCESS_SRC 0
#define RES_SLOT_SKYBOX_CUBEMAP 0
#define RES_SLOT_LINE_GIZMO_VERTEX 1
#define RES_SLOT_GIZMO_COLOR 0
#define RES_SLOT_SSAO_DEPTH 6
#define RES_SLOT_SSAO_NORMAL 7
#define RES_SLOT_SSAO_NOISE 8
#define RES_SLOT_SSAO_SAMPLES 9
#define RES_SLOT_SSAO_BLUR_INPUT 6
#define RES_SLOT_DEPTH_RESOLVE_INPUT 0

#define UAV_SLOT_DEPTH_RESOLVE_OUTPUT 0

#define SAMPLER_SLOT_CMP_PCF 0
#define SAMPLER_SLOT_CMP_POINT 1
#define SAMPLER_SLOT_AF16_CLAMP 2
#define SAMPLER_SLOT_AF8_CLAMP 3
#define SAMPLER_SLOT_AF4_CLAMP 4
#define SAMPLER_SLOT_AF2_CLAMP 4
#define SAMPLER_SLOT_TRI_CLAMP 5
#define SAMPLER_SLOT_BI_CLAMP 6
#define SAMPLER_SLOT_POINT_CLAMP 7
#define SAMPLER_SLOT_AF16_WRAP 8
#define SAMPLER_SLOT_AF8_WRAP 9
#define SAMPLER_SLOT_AF4_WRAP 10
#define SAMPLER_SLOT_AF2_WRAP 10
#define SAMPLER_SLOT_TRI_WRAP 11
#define SAMPLER_SLOT_BI_WRAP 12
#define SAMPLER_SLOT_POINT_WRAP 13

#define MAX_CASCADE_COUNT 4
#define MAX_PER_LIGHT_SHADOW_MAP_COUNT 6

#define BLEND_MODE_OPAQUE 0
#define BLEND_MODE_ALPHA_CLIP 1

#define SSAO_NOISE_TEX_DIM 4

#define DEPTH_RESOLVE_CS_THREADS_X 16
#define DEPTH_RESOLVE_CS_THREADS_Y 16
#define DEPTH_RESOLVE_CS_THREADS_Z 1

#define SKINNING_CS_THREADS 64


struct ShaderLight {
  row_major float4x4 shadowViewProjMatrices[MAX_PER_LIGHT_SHADOW_MAP_COUNT];

  float3 color;
  float intensity;

  float3 direction;
  int type;

  BOOL isCastingShadow;
  float range;
  float halfInnerAngleCos;
  float halfOuterAngleCos;

  float2 shadowAtlasCellOffsets[MAX_PER_LIGHT_SHADOW_MAP_COUNT];

  float shadowAtlasCellSizes[MAX_PER_LIGHT_SHADOW_MAP_COUNT];
  BOOL sampleShadowMap[MAX_PER_LIGHT_SHADOW_MAP_COUNT];

  float3 position;
  float depthBias;

  float normalBias;
  float3 pad;
};


struct ShaderMaterial {
  float3 albedo;
  float metallic;

  float roughness;
  float ao;
  float alphaThreshold;
  uint albedo_map_idx;

  uint metallic_map_idx;
  uint roughness_map_idx;
  uint ao_map_idx;
  uint normal_map_idx;

  uint opacity_map_idx;
  int blendMode;
  float2 pad;
};


struct ShaderLineGizmoVertexData {
  float3 from;
  uint colorIdx;
  float3 to;
  float pad;
};


struct ShaderPerFrameConstants {
  float3 ambientLightColor;
  int shadowCascadeCount;
  float2 screenSize;
  BOOL visualizeShadowCascades;
  int shadowFilteringMode;
};


struct ShaderPerViewConstants {
  row_major float4x4 viewMtx;
  row_major float4x4 projMtx;
  row_major float4x4 invProjMtx;
  row_major float4x4 viewProjMtx;
  float4 shadowCascadeSplitDistances;
  float3 viewPos;
  float pad;
};


struct ShaderPerDrawConstants {
  row_major float4x4 modelMtx;
  row_major float4x4 invTranspModelMtx;
};


struct DepthNormalDrawParams {
  uint pos_buf_idx;
  uint norm_buf_idx;
  uint tan_buf_idx;
  uint uv_buf_idx;
  uint mtl_idx;
  uint samp_idx;
  uint rt_idx;
  uint per_draw_cb_idx;
  uint per_view_cb_idx;
  uint per_frame_cb_idx;
};


struct DepthOnlyDrawParams {
  uint pos_buf_idx;
  uint uv_buf_idx;
  uint mtl_idx;
  uint samp_idx;
  uint rt_idx;
  uint per_draw_cb_idx;
  uint per_view_cb_idx;
};


struct DepthResolveDrawParams {
  uint in_tex_idx;
  uint out_tex_idx;
};


struct GizmoDrawParams {
  uint vertex_buf_idx;
  uint color_buf_idx;
  uint per_view_cb_idx;
};


struct ObjectDrawParams {
  uint pos_buf_idx;
  uint norm_buf_idx;
  uint tan_buf_idx;
  uint uv_buf_idx;

  uint mtl_idx;
  uint mtl_samp_idx;
  uint point_clamp_samp_idx;
  uint shadow_samp_idx;

  uint ssao_tex_idx;
  uint light_buf_idx;
  uint light_count;
  uint dir_shadow_arr_idx;

  uint punc_shadow_atlas_idx;
  uint per_draw_cb_idx;
  uint per_view_cb_idx;
  uint per_frame_cb_idx;
};


struct PostProcessDrawParams {
  uint in_tex_idx;
  float inv_gamma;
  uint bi_clamp_samp_idx;
};


struct SkyboxDrawParams {
  uint pos_buf_idx;
  uint per_view_cb_idx;
  uint cubemap_idx;
  uint samp_idx;
};


struct SsaoDrawParams {
  uint noise_tex_idx;
  uint depth_tex_idx;
  uint normal_tex_idx;
  uint samp_buf_idx;
  uint point_clamp_samp_idx;
  uint point_wrap_samp_idx;
  float radius;
  float bias;
  float power;
  int sample_count;
  uint per_view_cb_idx;
  uint per_frame_cb_idx;
};


struct SsaoBlurDrawParams {
  uint in_tex_idx;
  uint point_clamp_samp_idx;
};


struct VertexSkinningDrawParams {
  uint vtx_buf_idx;
  uint norm_buf_idx;
  uint tan_buf_idx;
  uint bone_weight_buf_idx;

  uint bone_idx_buf_idx;
  uint bone_buf_idx;
  uint skinned_vtx_buf_idx;
  uint skinned_norm_buf_idx;

  uint skinned_tan_buf_idx;
  uint vtx_count;
};


struct IrradianceDrawParams {
  row_major float4x4 view_proj_mtx;
  uint pos_buf_idx;
  uint cubemap_idx;
  uint samp_idx;
  uint rt_array_idx;
};


#ifdef __cplusplus
} // namespace sorcery
#endif

#endif

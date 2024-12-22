#include "scene_renderer.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <random>

#include "ShadowCascadeBoundary.hpp"
#include "../app.hpp"
#include "../MemoryAllocation.hpp"
#include "../ResourceManager.hpp"
#include "../Window.hpp"
#include "../Resources/Scene.hpp"
#include "../scene_objects/Entity.hpp"
#include "../scene_objects/TransformComponent.hpp"
#include "shaders/shader_interop.h"

#ifndef NDEBUG
#include "shaders/generated/Debug/depth_normal_ps.h"
#include "shaders/generated/Debug/depth_normal_vs.h"
#include "shaders/generated/Debug/depth_only_ps.h"
#include "shaders/generated/Debug/depth_only_vs.h"
#include "shaders/generated/Debug/depth_resolve_cs.h"
#include "shaders/generated/Debug/gizmos_line_vs.h"
#include "shaders/generated/Debug/gizmos_ps.h"
#include "shaders/generated/Debug/irradiance_ps.h"
#include "shaders/generated/Debug/irradiance_vs.h"
#include "shaders/generated/Debug/object_pbr_ps.h"
#include "shaders/generated/Debug/object_pbr_vs.h"
#include "shaders/generated/Debug/post_process_ps.h"
#include "shaders/generated/Debug/post_process_vs.h"
#include "shaders/generated/Debug/skybox_ps.h"
#include "shaders/generated/Debug/skybox_vs.h"
#include "shaders/generated/Debug/ssao_blur_ps.h"
#include "shaders/generated/Debug/ssao_main_ps.h"
#include "shaders/generated/Debug/ssao_vs.h"
#include "shaders/generated/Debug/vtx_skinning_cs.h"
#else
#include "shaders/generated/Release/depth_normal_ps.h"
#include "shaders/generated/Release/depth_normal_vs.h"
#include "shaders/generated/Release/depth_only_ps.h"
#include "shaders/generated/Release/depth_only_vs.h"
#include "shaders/generated/Release/depth_resolve_cs.h"
#include "shaders/generated/Release/gizmos_line_vs.h"
#include "shaders/generated/Release/gizmos_ps.h"
#include "shaders/generated/Release/irradiance_ps.h"
#include "shaders/generated/Release/irradiance_vs.h"
#include "shaders/generated/Release/object_pbr_ps.h"
#include "shaders/generated/Release/object_pbr_vs.h"
#include "shaders/generated/Release/post_process_ps.h"
#include "shaders/generated/Release/post_process_vs.h"
#include "shaders/generated/Release/skybox_ps.h"
#include "shaders/generated/Release/skybox_vs.h"
#include "shaders/generated/Release/ssao_blur_ps.h"
#include "shaders/generated/Release/ssao_main_ps.h"
#include "shaders/generated/Release/ssao_vs.h"
#include "shaders/generated/Release/vtx_skinning_cs.h"
#endif


namespace sorcery::rendering {
namespace {
// Returns view matrices looking at each face of a cube from the specified origin.
// The order of faces is +X, -X, +Y, -Y, +Z, -Z.
[[nodiscard]] auto MakeCubeFaceViewMatrices(Vector3 const& origin) noexcept {
  return std::array{
    Matrix4::LookTo(origin, Vector3::Right(), Vector3::Up()), // +X
    Matrix4::LookTo(origin, Vector3::Left(), Vector3::Up()), // -X
    Matrix4::LookTo(origin, Vector3::Up(), Vector3::Backward()), // +Y
    Matrix4::LookTo(origin, Vector3::Down(), Vector3::Forward()), // -Y
    Matrix4::LookTo(origin, Vector3::Forward(), Vector3::Up()), // +Z
    Matrix4::LookTo(origin, Vector3::Backward(), Vector3::Up()), // -Z
  };
}
}


auto SceneRenderer::CalculateCameraShadowCascadeBoundaries(CameraData const& cam_data,
                                                           ShadowParams const& shadow_params) ->
  ShadowCascadeBoundaries {
  auto const cam_near{cam_data.near_plane};
  auto const shadow_distance{std::min(cam_data.far_plane, shadow_params.distance)};
  auto const shadowed_frustum_depth{shadow_distance - cam_near};

  ShadowCascadeBoundaries boundaries;

  boundaries[0].nearClip = cam_near;

  for (auto i = 0; i < shadow_params.cascade_count - 1; i++) {
    boundaries[i + 1].nearClip = cam_near + shadow_params.normalized_cascade_splits[i] * shadowed_frustum_depth;
    boundaries[i].farClip = boundaries[i + 1].nearClip * 1.005f;
  }

  boundaries[shadow_params.cascade_count - 1].farClip = shadow_distance;

  for (int i = shadow_params.cascade_count; i < MAX_CASCADE_COUNT; i++) {
    boundaries[i].nearClip = std::numeric_limits<float>::infinity();
    boundaries[i].farClip = std::numeric_limits<float>::infinity();
  }

  return boundaries;
}


auto SceneRenderer::CullLights(Frustum const& frustum_ws, std::span<LightData const> const lights,
                               std::pmr::vector<unsigned>& visible_light_indices) -> void {
  visible_light_indices.clear();

  for (unsigned light_idx = 0; light_idx < static_cast<unsigned>(lights.size()); light_idx++) {
    switch (auto const light{lights[light_idx]}; light.type) {
      case LightComponent::Type::Directional: {
        visible_light_indices.emplace_back(light_idx);
        break;
      }

      case LightComponent::Type::Spot: {
        auto const light_vertices_ws{
          [light] {
            auto vertices{CalculateSpotLightLocalVertices(light.range, light.outer_angle)};

            for (auto const model_mtx_no_scale{light.local_to_world_mtx_no_scale}; auto& vertex : vertices) {
              vertex = Vector3{Vector4{vertex, 1} * model_mtx_no_scale};
            }

            return vertices;
          }()
        };

        if (frustum_ws.Intersects(AABB::FromVertices(light_vertices_ws))) {
          visible_light_indices.emplace_back(light_idx);
        }

        break;
      }

      case LightComponent::Type::Point: {
        if (BoundingSphere const bounds_ws{Vector3{light.position}, light.range}; frustum_ws.Intersects(bounds_ws)) {
          visible_light_indices.emplace_back(light_idx);
        }
        break;
      }
    }
  }
}


auto SceneRenderer::CullStaticSubmeshInstances(Frustum const& frustum_ws, std::span<MeshData const> const meshes,
                                               std::span<SubmeshData const> const submeshes,
                                               std::span<InstanceData const> const instances,
                                               std::pmr::vector<unsigned>& visible_static_submesh_instance_indices) ->
  void {
  visible_static_submesh_instance_indices.clear();

  for (unsigned i{0}; i < static_cast<unsigned>(instances.size()); i++) {
    auto const& instance{instances[i]};
    auto const& submesh{submeshes[instance.submesh_local_idx]};
    auto const& mesh{meshes[submesh.mesh_local_idx]};

    if (frustum_ws.Intersects(mesh.bounds.Transform(instance.local_to_world_mtx)) && frustum_ws.Intersects(
          submesh.bounds.Transform(instance.local_to_world_mtx))) {
      visible_static_submesh_instance_indices.emplace_back(i);
    }
  }
}


auto SceneRenderer::SetPerFrameConstants(ConstantBuffer<ShaderPerFrameConstants>& cb, int const rt_width,
                                         int const rt_height, Vector3 const& ambient_light,
                                         ShadowParams const& shadow_params) -> void {
  cb.Update(ShaderPerFrameConstants{
    .ambientLightColor = ambient_light, .shadowCascadeCount = shadow_params.cascade_count,
    .screenSize = Vector2{rt_width, rt_height}, .visualizeShadowCascades = shadow_params.visualize_cascades,
    .shadowFilteringMode = static_cast<int>(shadow_params.filtering_mode)
  });
}


auto SceneRenderer::SetPerViewConstants(ConstantBuffer<ShaderPerViewConstants>& cb, Matrix4 const& view_mtx,
                                        Matrix4 const& proj_mtx, ShadowCascadeBoundaries const& cascade_bounds,
                                        Vector3 const& view_pos) -> void {
  ShaderPerViewConstants data;
  data.viewMtx = view_mtx;
  data.projMtx = proj_mtx;
  data.invProjMtx = proj_mtx.Inverse();
  data.viewProjMtx = view_mtx * proj_mtx;
  data.viewPos = view_pos;

  for (auto i = 0; i < MAX_CASCADE_COUNT; i++) {
    data.shadowCascadeSplitDistances[i] = cascade_bounds[i].farClip;
  }

  cb.Update(data);
}


auto SceneRenderer::SetPerDrawConstants(ConstantBuffer<ShaderPerDrawConstants>& cb, Matrix4 const& model_mtx) -> void {
  cb.Update(ShaderPerDrawConstants{.modelMtx = model_mtx, .invTranspModelMtx = model_mtx.Inverse().Transpose()});
}


auto SceneRenderer::UpdatePunctualShadowAtlas(PunctualShadowAtlas& atlas,
                                              std::span<SceneRenderer::LightData const> const lights,
                                              std::span<unsigned const> visible_light_indices,
                                              SceneRenderer::CameraData const& cam_data,
                                              Matrix4 const& cam_view_proj_mtx, float const shadow_distance) -> void {
  struct LightCascadeIndex {
    int lightIdxIdx;
    int shadowIdx;
  };

  std::array lightIndexIndicesInCell{
    std::vector<LightCascadeIndex>{}, std::vector<LightCascadeIndex>{},
    std::vector<LightCascadeIndex>{}, std::vector<LightCascadeIndex>{}
  };

  auto const& camPos{cam_data.position};

  auto const determineScreenCoverage{
    [&camPos, &cam_view_proj_mtx](std::span<Vector3 const> const vertices) -> std::optional<int> {
      std::optional<int> cellIdx;

      if (auto const [worldMin, worldMax]{AABB::FromVertices(vertices)};
        worldMin[0] <= camPos[0] && worldMin[1] <= camPos[1] && worldMin[2] <= camPos[2] && worldMax[0] >= camPos[0] &&
        worldMax[1] >= camPos[1] && worldMax[2] >= camPos[2]) {
        cellIdx = 0;
      } else {
        Vector2 const bottomLeft{-1, -1};
        Vector2 const topRight{1, 1};

        Vector2 min{std::numeric_limits<float>::max()};
        Vector2 max{std::numeric_limits<float>::lowest()};

        for (auto& vertex : vertices) {
          Vector4 vertex4{vertex, 1};
          vertex4 *= cam_view_proj_mtx;
          auto const projected{Vector2{vertex4} / vertex4[3]};
          min = Clamp(Min(min, projected), bottomLeft, topRight);
          max = Clamp(Max(max, projected), bottomLeft, topRight);
        }

        auto const width{max[0] - min[0]};
        auto const height{max[1] - min[1]};

        auto const area{width * height};
        auto const coverage{area / 4};

        if (coverage >= 1) {
          cellIdx = 0;
        } else if (coverage >= 0.25f) {
          cellIdx = 1;
        } else if (coverage >= 0.0625f) {
          cellIdx = 2;
        } else if (coverage >= 0.015625f) {
          cellIdx = 3;
        }
      }

      return cellIdx;
    }
  };

  for (auto i = 0; i < static_cast<int>(visible_light_indices.size()); i++) {
    if (auto const light{lights[visible_light_indices[i]]};
      light.casts_shadow && (light.type == LightComponent::Type::Spot || light.type == LightComponent::Type::Point)) {
      Vector3 const& lightPos{light.position};
      float const lightRange{light.range};

      // Skip the light if its bounding sphere is farther than the shadow distance
      if (Vector3 const camToLightDir{Normalize(lightPos - camPos)}; Distance(lightPos - camToLightDir * lightRange,
                                                                       cam_data.position) > shadow_distance) {
        continue;
      }

      if (light.type == LightComponent::Type::Spot) {
        auto lightVertices{CalculateSpotLightLocalVertices(light.range, light.outer_angle)};

        for (auto const modelMtxNoScale{light.local_to_world_mtx_no_scale}; auto& vertex : lightVertices) {
          vertex = Vector3{Vector4{vertex, 1} * modelMtxNoScale};
        }

        if (auto const cellIdx{determineScreenCoverage(lightVertices)}) {
          lightIndexIndicesInCell[*cellIdx].emplace_back(i, 0);
        }
      } else if (light.type == LightComponent::Type::Point) {
        for (auto j = 0; j < 6; j++) {
          std::array static const faceBoundsRotations{
            Quaternion::FromAxisAngle(Vector3::Up(), ToRadians(90)), // +X
            Quaternion::FromAxisAngle(Vector3::Up(), ToRadians(-90)), // -X
            Quaternion::FromAxisAngle(Vector3::Right(), ToRadians(-90)), // +Y
            Quaternion::FromAxisAngle(Vector3::Right(), ToRadians(90)), // -Y
            Quaternion{}, // +Z
            Quaternion::FromAxisAngle(Vector3::Up(), ToRadians(180)) // -Z
          };

          std::array const shadowFrustumVertices{
            faceBoundsRotations[i].Rotate(Vector3{lightRange, lightRange, lightRange}) + lightPos,
            faceBoundsRotations[i].Rotate(Vector3{-lightRange, lightRange, lightRange}) + lightPos,
            faceBoundsRotations[i].Rotate(Vector3{-lightRange, -lightRange, lightRange}) + lightPos,
            faceBoundsRotations[i].Rotate(Vector3{lightRange, -lightRange, lightRange}) + lightPos, lightPos,
          };

          if (auto const cellIdx{determineScreenCoverage(shadowFrustumVertices)}) {
            lightIndexIndicesInCell[*cellIdx].emplace_back(i, j);
          }
        }
      }
    }
  }

  for (auto i = 0; i < 4; i++) {
    std::ranges::sort(lightIndexIndicesInCell[i],
      [&visible_light_indices, &camPos, &lights](LightCascadeIndex const lhs, LightCascadeIndex const rhs) {
        auto const leftLight{lights[visible_light_indices[lhs.lightIdxIdx]]};
        auto const rightLight{lights[visible_light_indices[rhs.lightIdxIdx]]};

        auto const leftLightPos{leftLight.position};
        auto const rightLightPos{rightLight.position};

        auto const leftDist{Distance(leftLightPos, camPos)};
        auto const rightDist{Distance(camPos, rightLightPos)};

        return leftDist > rightDist;
      });

    for (auto j = 0; j < atlas.GetCell(i).GetElementCount(); j++) {
      auto& subcell{atlas.GetCell(i).GetSubcell(j)};
      subcell.reset();

      if (lightIndexIndicesInCell[i].empty()) {
        continue;
      }

      auto const [lightIdxIdx, shadowIdx]{lightIndexIndicesInCell[i].back()};
      auto const light{lights[visible_light_indices[lightIdxIdx]]};
      lightIndexIndicesInCell[i].pop_back();

      if (light.type == LightComponent::Type::Spot) {
        auto const shadowViewMtx{Matrix4::LookTo(light.position, light.direction, Vector3::Up())};
        auto const shadowProjMtx{
          Matrix4::PerspectiveFov(ToRadians(light.outer_angle), 1.f, light.range, light.shadow_near_plane)
        };

        subcell.emplace(shadowViewMtx * shadowProjMtx, lightIdxIdx, shadowIdx);
      } else if (light.type == LightComponent::Type::Point) {
        auto const lightPos{light.position};

        auto const faceViewMatrices{MakeCubeFaceViewMatrices(lightPos)};
        auto const shadowViewMtx{faceViewMatrices[shadowIdx]};
        auto const shadowProjMtx{
          TransformProjectionMatrixForRendering(Matrix4::PerspectiveFov(ToRadians(90), 1, light.shadow_near_plane,
            light.range))
        };

        subcell.emplace(shadowViewMtx * shadowProjMtx, lightIdxIdx, shadowIdx);
      }
    }

    if (i + 1 < 4) {
      std::ranges::copy(lightIndexIndicesInCell[i], std::back_inserter(lightIndexIndicesInCell[i + 1]));
    }
  }
}


auto SceneRenderer::DrawDirectionalShadowMaps(FramePacket const& frame_packet,
                                              std::span<unsigned const> const visible_light_indices,
                                              CameraData const& cam_data, float rt_aspect, int const cascade_count,
                                              ShadowCascadeBoundaries const& shadow_cascade_boundaries,
                                              std::array<Matrix4, MAX_CASCADE_COUNT>& shadow_view_proj_matrices,
                                              graphics::CommandList& cmd) -> void {
  cmd.SetPipelineState(*shadow_pso_);
  cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, samp_idx), samp_af16_wrap_.Get());
  cmd.SetRenderTargets({}, dir_shadow_map_arr_->GetTex().get());
  cmd.Barrier({}, {}, std::array{
    graphics::TextureBarrier{
      D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_ACCESS_NO_ACCESS,
      D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE, D3D12_BARRIER_LAYOUT_UNDEFINED,
      D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE, dir_shadow_map_arr_->GetTex().get(), {0xffffffff, 0},
      D3D12_TEXTURE_BARRIER_FLAG_NONE
    }
  });
  cmd.ClearDepthStencil(*dir_shadow_map_arr_->GetTex(), D3D12_CLEAR_FLAG_DEPTH, 0.0f, 0, {});

  for (auto const lightIdx : visible_light_indices) {
    if (auto const light{frame_packet.light_data[lightIdx]};
      light.type == LightComponent::Type::Directional && light.casts_shadow) {
      float const camNear{cam_data.near_plane};
      float const camFar{cam_data.far_plane};

      enum FrustumVertex : int {
        FrustumVertex_NearTopRight    = 0,
        FrustumVertex_NearTopLeft     = 1,
        FrustumVertex_NearBottomLeft  = 2,
        FrustumVertex_NearBottomRight = 3,
        FrustumVertex_FarTopRight     = 4,
        FrustumVertex_FarTopLeft      = 5,
        FrustumVertex_FarBottomLeft   = 6,
        FrustumVertex_FarBottomRight  = 7,
      };

      // Order of vertices is CCW from top right, near first
      auto const frustumVertsWS{
        [&cam_data, rt_aspect, camNear, camFar] {
          std::array<Vector3, 8> ret;

          Vector3 const nearWorldForward{cam_data.position + cam_data.forward * camNear};
          Vector3 const farWorldForward{cam_data.position + cam_data.forward * camFar};

          switch (cam_data.type) {
            case Camera::Type::Perspective: {
              float const tanHalfFov{std::tan(ToRadians(cam_data.fov_vert_deg / 2.0f))};
              float const nearExtentY{camNear * tanHalfFov};
              float const nearExtentX{nearExtentY * rt_aspect};
              float const farExtentY{camFar * tanHalfFov};
              float const farExtentX{farExtentY * rt_aspect};

              ret[FrustumVertex_NearTopRight] = nearWorldForward + cam_data.right * nearExtentX + cam_data.up *
                                                nearExtentY;
              ret[FrustumVertex_NearTopLeft] = nearWorldForward - cam_data.right * nearExtentX + cam_data.up *
                                               nearExtentY;
              ret[FrustumVertex_NearBottomLeft] =
                nearWorldForward - cam_data.right * nearExtentX - cam_data.up * nearExtentY;
              ret[FrustumVertex_NearBottomRight] =
                nearWorldForward + cam_data.right * nearExtentX - cam_data.up * nearExtentY;
              ret[FrustumVertex_FarTopRight] = farWorldForward + cam_data.right * farExtentX + cam_data.up * farExtentY;
              ret[FrustumVertex_FarTopLeft] = farWorldForward - cam_data.right * farExtentX + cam_data.up * farExtentY;
              ret[FrustumVertex_FarBottomLeft] =
                farWorldForward - cam_data.right * farExtentX - cam_data.up * farExtentY;
              ret[FrustumVertex_FarBottomRight] =
                farWorldForward + cam_data.right * farExtentX - cam_data.up * farExtentY;
              break;
            }
            case Camera::Type::Orthographic: {
              float const extentX{cam_data.size_vert / 2.0f};
              float const extentY{extentX / rt_aspect};

              ret[FrustumVertex_NearTopRight] = nearWorldForward + cam_data.right * extentX + cam_data.up * extentY;
              ret[FrustumVertex_NearTopLeft] = nearWorldForward - cam_data.right * extentX + cam_data.up * extentY;
              ret[FrustumVertex_NearBottomLeft] = nearWorldForward - cam_data.right * extentX - cam_data.up * extentY;
              ret[FrustumVertex_NearBottomRight] = nearWorldForward + cam_data.right * extentX - cam_data.up * extentY;
              ret[FrustumVertex_FarTopRight] = farWorldForward + cam_data.right * extentX + cam_data.up * extentY;
              ret[FrustumVertex_FarTopLeft] = farWorldForward - cam_data.right * extentX + cam_data.up * extentY;
              ret[FrustumVertex_FarBottomLeft] = farWorldForward - cam_data.right * extentX - cam_data.up * extentY;
              ret[FrustumVertex_FarBottomRight] = farWorldForward + cam_data.right * extentX - cam_data.up * extentY;
              break;
            }
          }

          return ret;
        }()
      };

      auto const frustumDepth{camFar - camNear};

      for (auto cascadeIdx{0}; cascadeIdx < cascade_count; cascadeIdx++) {
        // cascade vertices in world space
        auto const cascadeVertsWS{
          [&frustumVertsWS, &shadow_cascade_boundaries, cascadeIdx, camNear, frustumDepth] {
            auto const [cascadeNear, cascadeFar]{shadow_cascade_boundaries[cascadeIdx]};

            float const cascadeNearNorm{(cascadeNear - camNear) / frustumDepth};
            float const cascadeFarNorm{(cascadeFar - camNear) / frustumDepth};

            std::array<Vector3, 8> ret;

            for (auto j = 0; j < 4; j++) {
              Vector3 const& from{frustumVertsWS[j]};
              Vector3 const& to{frustumVertsWS[j + 4]};

              ret[j] = Lerp(from, to, cascadeNearNorm);
              ret[j + 4] = Lerp(from, to, cascadeFarNorm);
            }

            return ret;
          }()
        };

        Vector3 cascadeCenterWS{Vector3::Zero()};

        for (Vector3 const& cascadeVertWS : cascadeVertsWS) {
          cascadeCenterWS += cascadeVertWS;
        }

        cascadeCenterWS /= 8.0f;

        auto sphereRadius{0.0f};

        for (Vector3 const& cascadeVertWS : cascadeVertsWS) {
          sphereRadius = std::max(sphereRadius, Distance(cascadeCenterWS, cascadeVertWS));
        }

        auto const shadowMapSize{dir_shadow_map_arr_->GetSize()};
        auto const worldUnitsPerTexel{sphereRadius * 2.0f / static_cast<float>(shadowMapSize)};

        Matrix4 shadowViewMtx{Matrix4::LookTo(Vector3::Zero(), light.direction, Vector3::Up())};
        cascadeCenterWS = Vector3{Vector4{cascadeCenterWS, 1} * shadowViewMtx};
        cascadeCenterWS /= worldUnitsPerTexel;
        cascadeCenterWS[0] = std::floor(cascadeCenterWS[0]);
        cascadeCenterWS[1] = std::floor(cascadeCenterWS[1]);
        cascadeCenterWS *= worldUnitsPerTexel;
        // shadowViewMtx is only rotation, transpose is its inverse
        cascadeCenterWS = Vector3{Vector4{cascadeCenterWS, 1} * shadowViewMtx.Transpose()};

        shadowViewMtx = Matrix4::LookTo(cascadeCenterWS, light.direction, Vector3::Up());
        auto const shadowProjMtx{
          TransformProjectionMatrixForRendering(Matrix4::OrthographicOffCenter(-sphereRadius, sphereRadius,
            sphereRadius, -sphereRadius, -sphereRadius - light.shadow_extension, sphereRadius))
        };

        shadow_view_proj_matrices[cascadeIdx] = shadowViewMtx * shadowProjMtx;

        cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, rt_idx), cascadeIdx);

        D3D12_VIEWPORT const shadowViewport{
          0, 0, static_cast<float>(shadowMapSize), static_cast<float>(shadowMapSize), 0, 1
        };

        D3D12_RECT const shadow_scissor{0, 0, static_cast<LONG>(shadowMapSize), static_cast<LONG>(shadowMapSize)};

        cmd.SetViewports(std::array{shadowViewport});
        cmd.SetScissorRects(std::array{shadow_scissor});

        auto& per_view_cb{AcquirePerViewConstantBuffer()};
        SetPerViewConstants(per_view_cb, shadowViewMtx, shadowProjMtx, ShadowCascadeBoundaries{}, Vector3{});
        cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, per_view_cb_idx),
          per_view_cb.GetBuffer()->GetConstantBuffer());

        Frustum const shadow_frustum_ws{shadow_view_proj_matrices[cascadeIdx]};

        std::pmr::vector<unsigned> visible_static_submesh_instance_indices;
        CullStaticSubmeshInstances(shadow_frustum_ws, frame_packet.mesh_data, frame_packet.submesh_data,
          frame_packet.instance_data, visible_static_submesh_instance_indices);

        for (auto const instance_idx : visible_static_submesh_instance_indices) {
          auto const& instance{frame_packet.instance_data[instance_idx]};
          auto const& submesh{frame_packet.submesh_data[instance.submesh_local_idx]};
          auto const& mesh{frame_packet.mesh_data[submesh.mesh_local_idx]};
          auto const& mtl_buf{frame_packet.buffers[submesh.mtl_buf_local_idx]};

          auto& per_draw_cb{AcquirePerDrawConstantBuffer()};
          SetPerDrawConstants(per_draw_cb, instance.local_to_world_mtx);

          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, pos_buf_idx),
            frame_packet.buffers[mesh.pos_buf_local_idx]->GetShaderResource());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, uv_buf_idx),
            frame_packet.buffers[mesh.uv_buf_local_idx]->GetShaderResource());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, mtl_idx), mtl_buf->GetConstantBuffer());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, per_draw_cb_idx),
            per_draw_cb.GetBuffer()->GetConstantBuffer());
          cmd.SetIndexBuffer(*frame_packet.buffers[mesh.idx_buf_local_idx], mesh.idx_format);
          cmd.DrawIndexedInstanced(submesh.index_count, 1, submesh.first_index, submesh.base_vertex, 0);
        }
      }

      break;
    }
  }
}


auto SceneRenderer::DrawPunctualShadowMaps(PunctualShadowAtlas const& atlas,
                                           SceneRenderer::FramePacket const& frame_packet,
                                           graphics::CommandList& cmd) -> void {
  cmd.SetPipelineState(*shadow_pso_);
  cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, rt_idx), 0);
  cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, samp_idx), samp_af16_wrap_.Get());
  cmd.SetRenderTargets({}, atlas.GetTex().get());

  cmd.Barrier({}, {}, std::array{
    graphics::TextureBarrier{
      D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_ACCESS_NO_ACCESS,
      D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE, D3D12_BARRIER_LAYOUT_UNDEFINED,
      D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE, punctual_shadow_atlas_->GetTex().get(), {0, 0},
      D3D12_TEXTURE_BARRIER_FLAG_NONE
    }
  });

  cmd.ClearDepthStencil(*atlas.GetTex(), D3D12_CLEAR_FLAG_DEPTH, 0, 0, {});

  auto const cell_size_norm{atlas.GetNormalizedElementSize()};

  for (auto i = 0; i < atlas.GetElementCount(); i++) {
    auto const& cell{atlas.GetCell(i)};
    auto const cell_offset_norm{atlas.GetNormalizedElementOffset(i)};
    auto const subcell_size{cell_size_norm * cell.GetNormalizedElementSize() * static_cast<float>(atlas.GetSize())};

    for (auto j = 0; j < cell.GetElementCount(); j++) {
      if (auto const& subcell{cell.GetSubcell(j)}) {
        auto const subcell_offset{
          (cell_offset_norm + cell.GetNormalizedElementOffset(j) * cell_size_norm) * static_cast<float>(atlas.GetSize())
        };

        D3D12_VIEWPORT const viewport{subcell_offset[0], subcell_offset[1], subcell_size, subcell_size, 0, 1};
        D3D12_RECT const scissor{
          static_cast<LONG>(subcell_offset[0]), static_cast<LONG>(subcell_offset[1]),
          static_cast<LONG>(subcell_offset[0] + subcell_size), static_cast<LONG>(subcell_offset[1] + subcell_size)
        };

        cmd.SetViewports(std::span{&viewport, 1});
        cmd.SetScissorRects(std::array{scissor});

        auto& per_view_cb{AcquirePerViewConstantBuffer()};
        SetPerViewConstants(per_view_cb, Matrix4::Identity(), subcell->shadowViewProjMtx, ShadowCascadeBoundaries{},
          Vector3{});
        cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, per_view_cb_idx),
          per_view_cb.GetBuffer()->GetConstantBuffer());

        Frustum const shadow_frustum_ws{subcell->shadowViewProjMtx};

        std::pmr::vector<unsigned> visible_static_submesh_instance_indices;
        CullStaticSubmeshInstances(shadow_frustum_ws, frame_packet.mesh_data, frame_packet.submesh_data,
          frame_packet.instance_data, visible_static_submesh_instance_indices);

        for (auto const instance_idx : visible_static_submesh_instance_indices) {
          auto const& instance{frame_packet.instance_data[instance_idx]};
          auto const& submesh{frame_packet.submesh_data[instance.submesh_local_idx]};
          auto const& mesh{frame_packet.mesh_data[submesh.mesh_local_idx]};
          auto const& mtl_buf{frame_packet.buffers[submesh.mtl_buf_local_idx]};

          auto& per_draw_cb{AcquirePerDrawConstantBuffer()};
          SetPerDrawConstants(per_draw_cb, instance.local_to_world_mtx);

          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, pos_buf_idx),
            frame_packet.buffers[mesh.pos_buf_local_idx]->GetShaderResource());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, uv_buf_idx),
            frame_packet.buffers[mesh.uv_buf_local_idx]->GetShaderResource());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, mtl_idx), mtl_buf->GetConstantBuffer());
          cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthOnlyDrawParams, per_draw_cb_idx),
            per_draw_cb.GetBuffer()->GetConstantBuffer());
          cmd.SetIndexBuffer(*frame_packet.buffers[mesh.idx_buf_local_idx], mesh.idx_format);
          cmd.DrawIndexedInstanced(submesh.index_count, 1, submesh.first_index, submesh.base_vertex, 0);
        }
      }
    }
  }
}


auto SceneRenderer::ClearGizmoDrawQueue() noexcept -> void {
  gizmo_colors_.clear();
  line_gizmo_vertex_data_.clear();
}


auto SceneRenderer::RecreateSsaoSamples(int const sample_count) noexcept -> void {
  ssao_samples_buffer_.Resize(sample_count);
  auto const ssao_samples{ssao_samples_buffer_.GetData()};

  std::uniform_real_distribution dist{0.0f, 1.0f};
  std::default_random_engine gen; // NOLINT(cert-msc51-cpp)

  for (auto i{0}; i < sample_count; i++) {
    Vector3 sample{dist(gen) * 2 - 1, dist(gen) * 2 - 1, dist(gen)};
    Normalize(sample);
    sample *= dist(gen);

    auto scale{static_cast<float>(i) / static_cast<float>(sample_count)};
    scale = std::lerp(0.1f, 1.0f, scale * scale);
    sample *= scale;

    ssao_samples[i] = Vector4{sample, 0};
  }
}


auto SceneRenderer::RecreatePipelines() -> void {
  CD3DX12_DEPTH_STENCIL_DESC1 const reverse_z_depth_stencil_write{
    TRUE, D3D12_DEPTH_WRITE_MASK_ALL, D3D12_COMPARISON_FUNC_GREATER, FALSE, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
    FALSE
  };

  CD3DX12_DEPTH_STENCIL_DESC1 const reverse_z_depth_stencil_read{
    TRUE, D3D12_DEPTH_WRITE_MASK_ZERO, D3D12_COMPARISON_FUNC_GREATER_EQUAL, FALSE, {}, {}, {}, {}, {}, {}, {}, {}, {},
    {}, FALSE
  };

  CD3DX12_DEPTH_STENCIL_DESC1 const disabled_depth_stencil{
    FALSE, D3D12_DEPTH_WRITE_MASK_ZERO, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, FALSE
  };

  CD3DX12_RASTERIZER_DESC const shadow_rasterizer_desc{
    D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, FALSE, -1, 0.f, -2.5f, TRUE, FALSE, FALSE, 0,
    D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF
  };

  CD3DX12_RASTERIZER_DESC const skybox_rasterizer_desc{
    D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_FRONT, FALSE, D3D12_DEFAULT_DEPTH_BIAS, D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
    D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS, TRUE, TRUE, FALSE, 0, {}
  };

  DXGI_SAMPLE_DESC const msaa_sample_desc{static_cast<UINT>(msaa_mode_), 0};
  CD3DX12_RT_FORMAT_ARRAY const render_target_format{D3D12_RT_FORMAT_ARRAY{{render_target_format_}, 1}};
  CD3DX12_RT_FORMAT_ARRAY const color_format{D3D12_RT_FORMAT_ARRAY{{color_buffer_format_}, 1}};
  CD3DX12_RT_FORMAT_ARRAY const ssao_format{D3D12_RT_FORMAT_ARRAY{{ssao_buffer_format_}, 1}};

  graphics::PipelineDesc const depth_normal_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{g_depth_normal_vs_bytes, ARRAYSIZE(g_depth_normal_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_depth_normal_ps_bytes, ARRAYSIZE(g_depth_normal_ps_bytes)},
    .depth_stencil_state = reverse_z_depth_stencil_write, .ds_format = depth_format_,
    .rt_formats = CD3DX12_RT_FORMAT_ARRAY{D3D12_RT_FORMAT_ARRAY{{normal_buffer_format_}, 1}},
    .sample_desc = msaa_sample_desc
  };

  depth_normal_pso_ = device_->CreatePipelineState(depth_normal_pso_desc, sizeof(DepthNormalDrawParams) / 4);

  graphics::PipelineDesc const shadow_pso_desc_{
    .vs = CD3DX12_SHADER_BYTECODE{g_depth_only_vs_bytes, ARRAYSIZE(g_depth_only_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_depth_only_ps_bytes, ARRAYSIZE(g_depth_only_ps_bytes)},
    .depth_stencil_state = reverse_z_depth_stencil_write, .ds_format = depth_format_,
    .rasterizer_state = shadow_rasterizer_desc
  };

  shadow_pso_ = device_->CreatePipelineState(shadow_pso_desc_, sizeof(DepthOnlyDrawParams) / 4);

  graphics::PipelineDesc const depth_resolve_pso_desc{
    .cs = CD3DX12_SHADER_BYTECODE{g_depth_resolve_cs_bytes, ARRAYSIZE(g_depth_resolve_cs_bytes)}
  };

  depth_resolve_pso_ = device_->CreatePipelineState(depth_resolve_pso_desc, sizeof(DepthResolveDrawParams) / 4);

  graphics::PipelineDesc const irradiance_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{g_irradiance_vs_bytes, ARRAYSIZE(g_irradiance_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_irradiance_ps_bytes, ARRAYSIZE(g_irradiance_ps_bytes)},
    .depth_stencil_state = disabled_depth_stencil, .rasterizer_state = skybox_rasterizer_desc,
    .rt_formats = CD3DX12_RT_FORMAT_ARRAY{D3D12_RT_FORMAT_ARRAY{{irradiance_map_format_}, 1}},
  };

  irradiance_pso_ = device_->CreatePipelineState(irradiance_pso_desc, sizeof(IrradianceDrawParams) / 4);

  graphics::PipelineDesc const line_gizmo_pso_desc{
    .primitive_topology_type = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE,
    .vs = CD3DX12_SHADER_BYTECODE{g_gizmos_line_vs_bytes, ARRAYSIZE(g_gizmos_line_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_gizmos_ps_bytes, ARRAYSIZE(g_gizmos_ps_bytes)},
    .depth_stencil_state = disabled_depth_stencil, .rt_formats = render_target_format
  };

  line_gizmo_pso_ = device_->CreatePipelineState(line_gizmo_pso_desc, sizeof(GizmoDrawParams) / 4);

  graphics::PipelineDesc const object_pso_depth_write_desc{
    .vs = CD3DX12_SHADER_BYTECODE{g_object_pbr_vs_bytes, ARRAYSIZE(g_object_pbr_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_object_pbr_ps_bytes, ARRAYSIZE(g_object_pbr_ps_bytes)},
    .depth_stencil_state = reverse_z_depth_stencil_write, .ds_format = depth_format_, .rt_formats = color_format,
    .sample_desc = msaa_sample_desc,
  };

  object_pso_depth_write_ = device_->CreatePipelineState(object_pso_depth_write_desc, sizeof(ObjectDrawParams) / 4);

  graphics::PipelineDesc const object_pso_depth_read_desc{
    .vs = CD3DX12_SHADER_BYTECODE{g_object_pbr_vs_bytes, ARRAYSIZE(g_object_pbr_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{g_object_pbr_ps_bytes, ARRAYSIZE(g_object_pbr_ps_bytes)},
    .depth_stencil_state = reverse_z_depth_stencil_read, .ds_format = depth_format_, .rt_formats = color_format,
    .sample_desc = msaa_sample_desc,
  };

  object_pso_depth_read_ = device_->CreatePipelineState(object_pso_depth_read_desc, sizeof(ObjectDrawParams) / 4);

  graphics::PipelineDesc const post_process_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{&g_post_process_vs_bytes, ARRAYSIZE(g_post_process_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{&g_post_process_ps_bytes, ARRAYSIZE(g_post_process_ps_bytes)},
    .depth_stencil_state = disabled_depth_stencil, .rt_formats = render_target_format,
  };

  post_process_pso_ = device_->CreatePipelineState(post_process_pso_desc, sizeof(PostProcessDrawParams) / 4);

  graphics::PipelineDesc const skybox_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{&g_skybox_vs_bytes, ARRAYSIZE(g_skybox_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{&g_skybox_ps_bytes, ARRAYSIZE(g_skybox_ps_bytes)},
    .depth_stencil_state = reverse_z_depth_stencil_read,
    .ds_format = depth_format_,
    .rasterizer_state = skybox_rasterizer_desc,
    .rt_formats = color_format, .sample_desc = msaa_sample_desc
  };

  skybox_pso_ = device_->CreatePipelineState(skybox_pso_desc, sizeof(SkyboxDrawParams) / 4);

  graphics::PipelineDesc const ssao_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{&g_ssao_vs_bytes, ARRAYSIZE(g_ssao_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{&g_ssao_main_ps_bytes, ARRAYSIZE(g_ssao_main_ps_bytes)},
    .depth_stencil_state = disabled_depth_stencil, .rt_formats = ssao_format,
  };

  ssao_pso_ = device_->CreatePipelineState(ssao_pso_desc, sizeof(SsaoDrawParams) / 4);

  graphics::PipelineDesc const ssao_blur_pso_desc{
    .vs = CD3DX12_SHADER_BYTECODE{&g_ssao_vs_bytes, ARRAYSIZE(g_ssao_vs_bytes)},
    .ps = CD3DX12_SHADER_BYTECODE{&g_ssao_blur_ps_bytes, ARRAYSIZE(g_ssao_blur_ps_bytes)},
    .depth_stencil_state = disabled_depth_stencil, .rt_formats = ssao_format
  };

  ssao_blur_pso_ = device_->CreatePipelineState(ssao_blur_pso_desc, sizeof(SsaoBlurDrawParams) / 4);

  graphics::PipelineDesc const vtx_skinning_pso_desc{
    .cs = CD3DX12_SHADER_BYTECODE{g_vtx_skinning_cs_bytes, ARRAYSIZE(g_vtx_skinning_cs_bytes)}
  };

  vtx_skinning_pso_ = device_->CreatePipelineState(vtx_skinning_pso_desc, sizeof(VertexSkinningDrawParams) / 4);
}


auto SceneRenderer::CreatePerViewConstantBuffers(UINT const count) -> void {
  per_view_cbs_.reserve(per_view_cbs_.size() + count);

  for (UINT i{0}; i < count; i++) {
    auto& arr{per_view_cbs_.emplace_back()};

    for (UINT j{0}; j < RenderManager::GetMaxFramesInFlight(); j++) {
      if (auto opt{ConstantBuffer<ShaderPerViewConstants>::New(*device_, true)}) {
        arr[j] = std::move(*opt);
      }
    }
  }
}


auto SceneRenderer::CreatePerDrawConstantBuffers(UINT const count) -> void {
  per_draw_cbs_.reserve(per_draw_cbs_.size() + count);

  for (UINT i{0}; i < count; i++) {
    auto& arr{per_draw_cbs_.emplace_back()};

    for (UINT j{0}; j < RenderManager::GetMaxFramesInFlight(); j++) {
      if (auto opt{ConstantBuffer<ShaderPerDrawConstants>::New(*device_, true)}) {
        arr[j] = std::move(*opt);
      }
    }
  }
}


auto SceneRenderer::AcquirePerViewConstantBuffer() -> ConstantBuffer<ShaderPerViewConstants>& {
  if (next_per_view_cb_idx_ >= per_view_cbs_.size()) {
    CreatePerViewConstantBuffers(1);
  }

  return per_view_cbs_[next_per_view_cb_idx_++][render_manager_->GetCurrentFrameIndex()];
}


auto SceneRenderer::AcquirePerDrawConstantBuffer() -> ConstantBuffer<ShaderPerDrawConstants>& {
  if (next_per_draw_cb_idx_ >= per_draw_cbs_.size()) {
    CreatePerDrawConstantBuffers(1);
  }

  return per_draw_cbs_[next_per_draw_cb_idx_++][render_manager_->GetCurrentFrameIndex()];
}


auto SceneRenderer::OnWindowSize(Extent2D<std::uint32_t> const size) -> void {
  if (size.width != 0 && size.height != 0) {
    RenderTarget::Desc desc{main_rt_->GetDesc()};
    desc.width = size.width;
    desc.height = size.height;
    main_rt_ = RenderTarget::New(*device_, desc);
  }
}


SceneRenderer::SceneRenderer(Window& window, graphics::GraphicsDevice& device, RenderManager& render_manager) :
  render_manager_{&render_manager},
  window_{&window},
  device_{&device} {
  for (auto& buf : light_buffers_) {
    buf = StructuredBuffer<ShaderLight>::New(*device_, *render_manager_, true);
  }

  gizmo_color_buffer_ = StructuredBuffer<Vector4>::New(*device_, *render_manager_, true);

  line_gizmo_vertex_data_buffer_ = StructuredBuffer<ShaderLineGizmoVertexData>::New(*device_, *render_manager_, true);

  main_rt_ = RenderTarget::New(*device_, RenderTarget::Desc{
    static_cast<UINT>(window_->GetClientAreaSize().width), static_cast<UINT>(window_->GetClientAreaSize().height),
    DXGI_FORMAT_R8G8B8A8_UNORM, std::nullopt, 1, L"Main RT", false
  });

  dir_shadow_map_arr_ = std::make_unique<DirectionalShadowMapArray>(device_.Get(), depth_format_, 4096);
  dir_shadow_map_arr_->GetTex()->SetDebugName(L"Directional Shadow Map Array");

  punctual_shadow_atlas_ = std::make_unique<PunctualShadowAtlas>(device_.Get(), depth_format_, 4096);
  punctual_shadow_atlas_->GetTex()->SetDebugName(L"Punctual Shadow Atlas");

  RecreatePipelines();

  for (auto& cb : per_frame_cbs_) {
    if (auto opt{ConstantBuffer<ShaderPerFrameConstants>::New(*device_, true)}) {
      cb = std::move(*opt);
    }
  }

  CreatePerViewConstantBuffers(1);
  CreatePerDrawConstantBuffers(100);

  samp_cmp_pcf_ge_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_GREATER_EQUAL, {},
    0, 0
  });

  samp_cmp_pcf_le_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_LESS_EQUAL, {}, 0, 0
  });

  samp_cmp_point_ge_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_COMPARISON_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_GREATER_EQUAL, {}, 0, 0
  });

  samp_cmp_point_le_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_COMPARISON_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_LESS_EQUAL, {}, 0, 0
  });

  samp_af16_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 16, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af8_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 8, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af4_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 4, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af2_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 2, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_tri_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_bi_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_point_clamp_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
    D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af16_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 16, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af8_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 8, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af4_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 4, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_af2_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 2, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_tri_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_bi_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  samp_point_wrap_ = device_->CreateSampler(D3D12_SAMPLER_DESC{
    D3D12_FILTER_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP,
    D3D12_TEXTURE_ADDRESS_MODE_WRAP, 0, 1, D3D12_COMPARISON_FUNC_NEVER, {}, 0, std::numeric_limits<float>::max()
  });

  window_size_event_listener_ = window_->OnWindowSize.add_listener([this](Extent2D<unsigned> const size) {
    OnWindowSize(size);
  });

  ssao_samples_buffer_ = StructuredBuffer<Vector4>::New(*device_, *render_manager_, true);
  RecreateSsaoSamples(ssao_params_.sample_count);

  ssao_noise_tex_ = device_->CreateTexture(graphics::TextureDesc{
    graphics::TextureDimension::k2D, SSAO_NOISE_TEX_DIM, SSAO_NOISE_TEX_DIM, 1, 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 1,
    false, false, true, false
  }, D3D12_HEAP_TYPE_DEFAULT, D3D12_BARRIER_LAYOUT_COPY_DEST, nullptr);
  ssao_noise_tex_->SetDebugName(L"SSAO Noise");

  std::vector<Vector4> ssao_noise;
  std::uniform_real_distribution dist{0.0f, 1.0f};
  std::default_random_engine gen; // NOLINT(cert-msc51-cpp)

  for (auto i{0}; i < SSAO_NOISE_TEX_DIM * SSAO_NOISE_TEX_DIM; i++) {
    ssao_noise.emplace_back(dist(gen) * 2 - 1, dist(gen) * 2 - 1, 0, 0);
  }

  render_manager_->UpdateTexture(*ssao_noise_tex_, 0, std::array{
    D3D12_SUBRESOURCE_DATA{
      ssao_noise.data(), SSAO_NOISE_TEX_DIM * sizeof(Vector4), SSAO_NOISE_TEX_DIM * SSAO_NOISE_TEX_DIM * sizeof(Vector4)
    }
  });


  white_tex_ = device_->CreateTexture(graphics::TextureDesc{
    graphics::TextureDimension::k2D, 1, 1, 1, 1, DXGI_FORMAT_R8G8B8A8_UNORM, 1, false, false, true, false
  }, D3D12_HEAP_TYPE_DEFAULT, D3D12_BARRIER_LAYOUT_COPY_DEST, nullptr);

  std::array<std::uint8_t, 4> constexpr white_tex_data{255, 255, 255, 255};

  render_manager_->UpdateTexture(*white_tex_, 0, std::array{
    D3D12_SUBRESOURCE_DATA{white_tex_data.data(), sizeof(white_tex_data), sizeof(white_tex_data)}
  });
}


SceneRenderer::~SceneRenderer() {
  window_->OnWindowSize.remove_listener(window_size_event_listener_);
}


auto SceneRenderer::ExtractCurrentState() -> void {
  auto& packet{frame_packets_[render_manager_->GetCurrentFrameIndex()]};

  packet.buffers.clear();
  packet.textures.clear();
  packet.light_data.clear();
  packet.mesh_data.clear();
  packet.submesh_data.clear();
  packet.instance_data.clear();
  packet.cam_data.clear();
  packet.render_targets.clear();
  packet.anim_pos_keys.clear();
  packet.anim_rot_keys.clear();
  packet.anim_scaling_keys.clear();
  packet.node_anim_data.clear();
  packet.skeleton_node_data.clear();
  packet.bone_data.clear();
  packet.skinned_mesh_data.clear();

  packet.light_data.reserve(lights_.size());

  for (auto const light : lights_) {
    packet.light_data.emplace_back(light->GetColor(), light->GetIntensity(), light->GetDirection(),
      light->GetEntity()->GetTransform().GetWorldPosition(), light->GetType(), light->GetRange(),
      light->GetInnerAngle(),
      light->GetOuterAngle(), light->IsCastingShadow(), light->GetShadowNearPlane(), light->GetShadowNormalBias(),
      light->GetShadowDepthBias(), light->GetShadowExtension(),
      light->GetEntity()->GetTransform().CalculateLocalToWorldMatrixWithoutScale());
  }

  packet.mesh_data.reserve(static_mesh_components_.size());

  auto const find_or_emplace_back_buffer{
    [&packet](graphics::SharedDeviceChildHandle<graphics::Buffer> const& buf) -> unsigned {
      unsigned idx;

      if (auto const it{std::ranges::find(packet.buffers, buf)}; it != std::ranges::end(packet.buffers)) {
        idx = static_cast<unsigned>(it - packet.buffers.begin());
      } else {
        idx = static_cast<unsigned>(packet.buffers.size());
        packet.buffers.emplace_back(buf);
      }

      return idx;
    }
  };

  auto const find_or_emplace_back_texture{
    [&packet](graphics::SharedDeviceChildHandle<graphics::Texture> const& tex) -> unsigned {
      unsigned idx;

      if (auto const it{std::ranges::find(packet.textures, tex)}; it != std::ranges::end(packet.textures)) {
        idx = static_cast<unsigned>(it - packet.textures.begin());
      } else {
        idx = static_cast<unsigned>(packet.textures.size());
        packet.textures.emplace_back(tex);
      }

      return idx;
    }
  };

  auto const extract_from_mesh_comp{
    [&find_or_emplace_back_buffer, &packet, &find_or_emplace_back_texture](MeshComponentBase const* const comp) {
      auto const mesh{comp->GetMesh()};

      if (!mesh) {
        return;
      }

      auto const pos_buf_local_idx{find_or_emplace_back_buffer(mesh->GetPositionBuffer())};
      auto const norm_buf_local_idx{find_or_emplace_back_buffer(mesh->GetNormalBuffer())};
      auto const tan_buf_local_idx{find_or_emplace_back_buffer(mesh->GetTangentBuffer())};
      auto const uv_buf_local_idx{find_or_emplace_back_buffer(mesh->GetUvBuffer())};
      auto const idx_buf_local_idx{find_or_emplace_back_buffer(mesh->GetIndexBuffer())};
      auto const idx_format{mesh->GetIndexFormat()};
      packet.mesh_data.emplace_back(pos_buf_local_idx, norm_buf_local_idx, tan_buf_local_idx, uv_buf_local_idx,
        idx_buf_local_idx, static_cast<unsigned>(mesh->GetVertexCount()), mesh->GetBounds(), idx_format);

      packet.submesh_data.reserve(packet.submesh_data.size() + mesh->GetSubmeshCount());

      for (auto const& submesh : mesh->GetSubMeshes()) {
        auto const mtl{comp->GetMaterials()[submesh.material_index]};

        if (!mtl) {
          continue;
        }

        auto const mtl_buf_local_idx{find_or_emplace_back_buffer(mtl->GetBuffer())};

        for (auto const tex : {
               mtl->GetAlbedoMap(), mtl->GetMetallicMap(), mtl->GetRoughnessMap(), mtl->GetAoMap(), mtl->GetNormalMap(),
               mtl->GetOpacityMask()
             }) {
          if (tex) {
            find_or_emplace_back_texture(tex->GetTex());
          }
        }

        packet.submesh_data.emplace_back(static_cast<unsigned>(packet.mesh_data.size() - 1), submesh.base_vertex,
          submesh.first_index, submesh.index_count, mtl_buf_local_idx, submesh.bounds);

        packet.instance_data.emplace_back(static_cast<unsigned>(packet.submesh_data.size() - 1),
          comp->GetEntity()->GetTransform().GetLocalToWorldMatrix());
      }
    }
  };

  std::ranges::for_each(static_mesh_components_, extract_from_mesh_comp);

  std::ranges::for_each(skinned_mesh_components_,
    [&extract_from_mesh_comp, &packet, this](SkinnedMeshComponent const* const comp) {
      auto const mesh{comp->GetMesh()};

      if (!mesh) {
        return;
      }

      extract_from_mesh_comp(comp);

      auto const anim{comp->GetCurrentAnimation()};

      if (!anim) {
        return;
      }

      // TODO add proper skinned mesh culling with GPU culling

      AABB const inf_aabb{
        Vector3{-std::numeric_limits<float>::infinity()},
        Vector3{std::numeric_limits<float>::infinity()}
      };

      // Set mesh AABB to infinity to prevent culling
      packet.mesh_data.back().bounds = inf_aabb;

      // Set submesh AABBs to infinity to prevent culling
      for (auto i{std::ssize(packet.submesh_data) - 1};
           i >= 0 && packet.submesh_data[i].mesh_local_idx == std::ssize(packet.mesh_data) - 1; i--) {
        packet.submesh_data[i].bounds = inf_aabb;
      }

      packet.buffers.emplace_back(mesh->GetBoneWeightBuffer());
      auto const bone_weight_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      packet.buffers.emplace_back(mesh->GetBoneIndexBuffer());
      auto const bone_index_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      packet.buffers.emplace_back(comp->GetSkinnedVertexBuffers()[render_manager_->GetCurrentFrameIndex()]);
      auto const skinned_pos_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      packet.buffers.emplace_back(comp->GetSkinnedNormalBuffers()[render_manager_->GetCurrentFrameIndex()]);
      auto const skinned_norm_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      packet.buffers.emplace_back(comp->GetSkinnedTangentBuffers()[render_manager_->GetCurrentFrameIndex()]);
      auto const skinned_tan_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      packet.buffers.emplace_back(comp->GetBoneMatrixBuffers()[render_manager_->GetCurrentFrameIndex()]);
      auto const bone_mtx_buf_local_idx{static_cast<unsigned>(packet.buffers.size() - 1)};

      // Switch the original and skinned buffer indices so that the renderer can treat the skinned mesh as static after
      // the skinning is done

      auto const orig_pos_buf_local_idx{packet.mesh_data.back().pos_buf_local_idx};
      auto const orig_norm_buf_local_idx{packet.mesh_data.back().norm_buf_local_idx};
      auto const orig_tan_buf_local_idx{packet.mesh_data.back().tan_buf_local_idx};

      packet.mesh_data.back().pos_buf_local_idx = skinned_pos_buf_local_idx;
      packet.mesh_data.back().norm_buf_local_idx = skinned_norm_buf_local_idx;
      packet.mesh_data.back().tan_buf_local_idx = skinned_tan_buf_local_idx;

      // Extract animation data

      auto const node_anim_begin_local_idx{static_cast<unsigned>(packet.node_anim_data.size())};

      for (auto const& [pos_keys, rot_keys, scaling_keys, node_idx] : anim->node_anims) {
        auto const pos_key_begin_local_idx{static_cast<unsigned>(packet.anim_pos_keys.size())};
        auto const rot_key_begin_local_idx{static_cast<unsigned>(packet.anim_rot_keys.size())};
        auto const scaling_key_begin_local_idx{static_cast<unsigned>(packet.anim_scaling_keys.size())};

        std::ranges::copy(pos_keys, std::back_inserter(packet.anim_pos_keys));
        std::ranges::copy(rot_keys, std::back_inserter(packet.anim_rot_keys));
        std::ranges::copy(scaling_keys, std::back_inserter(packet.anim_scaling_keys));

        packet.node_anim_data.emplace_back(pos_key_begin_local_idx, static_cast<unsigned>(pos_keys.size()),
          rot_key_begin_local_idx, static_cast<unsigned>(rot_keys.size()), scaling_key_begin_local_idx,
          static_cast<unsigned>(scaling_keys.size()), node_idx);
      }

      // Extract skeleton data

      auto const skeleton_begin_local_idx{static_cast<unsigned>(packet.skeleton_node_data.size())};
      std::ranges::transform(mesh->GetSkeleton(), std::back_inserter(packet.skeleton_node_data),
        [&comp](SkeletonNode const& node) {
          return SkeletonNodeData{node.transform, node.parent_idx};
        });

      // Extract bone data

      auto const bone_begin_local_idx{static_cast<unsigned>(packet.bone_data.size())};
      std::ranges::transform(mesh->GetBones(), std::back_inserter(packet.bone_data),
        [&comp](Bone const& bone) {
          return BoneData{bone.offset_mtx, bone.skeleton_node_idx};
        });

      packet.skinned_mesh_data.emplace_back(static_cast<unsigned>(packet.mesh_data.size() - 1),
        orig_pos_buf_local_idx, orig_norm_buf_local_idx, orig_tan_buf_local_idx, bone_weight_buf_local_idx,
        bone_index_buf_local_idx, bone_mtx_buf_local_idx, comp->GetCurrentAnimationTime(), node_anim_begin_local_idx,
        static_cast<unsigned>(anim->node_anims.size()), skeleton_begin_local_idx,
        static_cast<unsigned>(mesh->GetSkeleton().size()), bone_begin_local_idx,
        static_cast<unsigned>(mesh->GetBones().size()));
    });

  auto const find_or_emplace_back_rt{
    [&packet](std::shared_ptr<RenderTarget> const& rt) -> unsigned {
      unsigned idx;

      if (auto const it{std::ranges::find(packet.render_targets, rt)}; it != std::ranges::end(packet.render_targets)) {
        idx = static_cast<unsigned>(it - packet.render_targets.begin());
      } else {
        idx = static_cast<unsigned>(packet.render_targets.size());
        packet.render_targets.emplace_back(rt);
      }

      return idx;
    }
  };

  packet.render_targets.emplace_back(rt_override_ ? rt_override_ : main_rt_); // The global RT is always at index 0!

  packet.cam_data.reserve(cameras_.size());

  for (auto const cam : cameras_) {
    unsigned rt_local_idx;

    if (auto const rt{cam->GetRenderTarget()}) {
      rt_local_idx = find_or_emplace_back_rt(rt);
    } else {
      rt_local_idx = 0; // The global RT is always at index 0!
    }

    packet.cam_data.emplace_back(cam->GetPosition(), cam->GetRightAxis(), cam->GetUpAxis(), cam->GetForwardAxis(),
      cam->GetNearClipPlane(), cam->GetFarClipPlane(), cam->GetType(), cam->GetVerticalPerspectiveFov(),
      cam->GetVerticalOrthographicSize(), cam->GetViewport(), rt_local_idx);
  }

  packet.gizmo_colors = gizmo_colors_;
  packet.line_gizmo_vertex_data = line_gizmo_vertex_data_;

  // This has to be cleared here so that the game, while rendering the frame,
  // can queue new gizmos without a race condition.
  ClearGizmoDrawQueue();

  packet.msaa_mode = msaa_mode_;
  packet.ssao_params = ssao_params_;
  packet.shadow_params = shadow_params_;

  packet.inv_gamma = inv_gamma_;

  packet.depth_normal_pre_pass_enabled = depth_normal_pre_pass_enabled_;
  packet.ssao_enabled = ssao_enabled_;

  packet.color_buffer_format = color_buffer_format_;

  packet.background_color = {0, 0, 0, 1};
  packet.skybox_cubemap = nullptr;
  packet.irradiance_tex = nullptr;
  packet.draw_irradiance_map = false;

  if (auto const active_scene{Scene::GetActiveScene()}) {
    packet.ambient_light = active_scene->GetAmbientLightVector();

    if (active_scene->GetSkyMode() == SkyMode::Color) {
      auto const sky_color{active_scene->GetSkyColor()};
      packet.background_color = {sky_color[0], sky_color[1], sky_color[2], 1.0f};
    }

    if (active_scene->GetSkyMode() == SkyMode::Skybox) {
      if (auto const cubemap{active_scene->GetSkybox()}) {
        packet.skybox_cubemap = cubemap->GetTex();

        if (!irradiance_rt_ || irradiance_src_guid_ != cubemap->GetGuid()) {
          irradiance_rt_ = RenderTarget::New(*device_, RenderTarget::Desc{
            .width = cubemap->GetTex()->GetDesc().width,
            .height = cubemap->GetTex()->GetDesc().height,
            .color_format = irradiance_map_format_,
            .debug_name = L"Irradiance RT",
            .dimension = graphics::TextureDimension::kCube,
            .depth_or_array_size = 6
          });

          irradiance_src_guid_ = cubemap->GetGuid();
          packet.draw_irradiance_map = true;
        }

        packet.irradiance_tex = irradiance_rt_->GetColorTex();
      }
    }
  }

  packet.shadow_pso = shadow_pso_;
  packet.depth_normal_pso = depth_normal_pso_;
  packet.depth_resolve_pso = depth_resolve_pso_;
  packet.irradiance_pso = irradiance_pso_;
  packet.line_gizmo_pso = line_gizmo_pso_;
  packet.object_pso_depth_write = object_pso_depth_write_;
  packet.object_pso_depth_read = object_pso_depth_read_;
  packet.post_process_pso = post_process_pso_;
  packet.skybox_pso = skybox_pso_;
  packet.ssao_pso = ssao_pso_;
  packet.ssao_blur_pso = ssao_blur_pso_;
}


auto SceneRenderer::Render() -> void {
  next_per_draw_cb_idx_ = 0;
  next_per_view_cb_idx_ = 0;

  auto const frame_idx{render_manager_->GetCurrentFrameIndex()};

  auto& frame_packet{frame_packets_[frame_idx]};

  gizmo_color_buffer_.Resize(static_cast<int>(std::ssize(frame_packet.gizmo_colors)));
  std::ranges::copy(frame_packet.gizmo_colors, std::begin(gizmo_color_buffer_.GetData()));

  line_gizmo_vertex_data_buffer_.Resize(static_cast<int>(std::ssize(frame_packet.line_gizmo_vertex_data)));
  std::ranges::copy(frame_packet.line_gizmo_vertex_data, std::begin(line_gizmo_vertex_data_buffer_.GetData()));

  // Transitions render targets, dispatches skinning, and draws irradiance map
  auto& prepare_cmd{render_manager_->AcquireCommandList()};
  prepare_cmd.Begin(nullptr);

  std::vector<graphics::TextureBarrier> cam_rt_barriers;
  cam_rt_barriers.reserve(frame_packet.render_targets.size());

  std::ranges::transform(frame_packet.render_targets, std::back_inserter(cam_rt_barriers),
    [](std::shared_ptr<RenderTarget> const& rt) {
      return graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
        D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
        rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      };
    });

  prepare_cmd.Barrier({}, {}, cam_rt_barriers);

  std::ranges::for_each(frame_packet.render_targets, [&prepare_cmd](std::shared_ptr<RenderTarget> const& rt) {
    prepare_cmd.ClearRenderTarget(*rt->GetColorTex(), rt->GetDesc().color_clear_value, {});
  });

  if (!frame_packet.skinned_mesh_data.empty()) {
    prepare_cmd.SetPipelineState(*vtx_skinning_pso_);
  }

  for (auto& [mesh_data_local_idx, original_vertex_buf_local_idx, original_normal_buf_local_idx,
         original_tangent_buf_local_idx, bone_weight_buf_local_idx, bone_index_buf_local_idx, bone_matrix_buf_local_idx,
         cur_animation_time, node_anim_begin_local_idx, node_anim_count, skeleton_begin_local_idx, skeleton_size,
         bone_begin_local_idx, bone_count] : frame_packet.skinned_mesh_data) {
    // Skip skinning when we are sitting at 0 time.
    // This happens for example in the editor scene view.
    if (cur_animation_time == 0) {
      prepare_cmd.CopyBuffer(*frame_packet.buffers[frame_packet.mesh_data[mesh_data_local_idx].pos_buf_local_idx],
        *frame_packet.buffers[original_vertex_buf_local_idx]);
      prepare_cmd.CopyBuffer(*frame_packet.buffers[frame_packet.mesh_data[mesh_data_local_idx].norm_buf_local_idx],
        *frame_packet.buffers[original_normal_buf_local_idx]);
      prepare_cmd.CopyBuffer(*frame_packet.buffers[frame_packet.mesh_data[mesh_data_local_idx].tan_buf_local_idx],
        *frame_packet.buffers[original_tangent_buf_local_idx]);
      continue;
    }

    // Compute local node transforms

    for (unsigned i{0}; i < node_anim_count; i++) {
      auto const& [pos_key_begin_local_idx, pos_key_count, rot_key_begin_local_idx, rot_key_count,
        scaling_key_begin_local_idx, scaling_key_count, node_idx]{
        frame_packet.node_anim_data[node_anim_begin_local_idx + i]
      };

      Vector3 pos{};
      Quaternion rot{};
      Vector3 scale{1};

      auto const calc_interpolation_factor{
        [](float const from_time, float const to_time, float const current_time) {
          return (current_time - from_time) / (to_time - from_time);
        }
      };

      if (pos_key_count == 1) {
        pos = frame_packet.anim_pos_keys[pos_key_begin_local_idx].value;
      } else {
        for (unsigned j{0}; j < pos_key_count; j++) {
          if (auto const& [timestamp, value]{frame_packet.anim_pos_keys[pos_key_begin_local_idx + j]};
            timestamp > cur_animation_time) {
            auto const& [prev_timestamp, prev_value]{frame_packet.anim_pos_keys[pos_key_begin_local_idx + j - 1]};
            pos = Lerp(prev_value, value, calc_interpolation_factor(prev_timestamp, timestamp, cur_animation_time));
            break;
          }
        }
      }

      if (rot_key_count == 1) {
        rot = frame_packet.anim_rot_keys[rot_key_begin_local_idx].value;
      } else {
        for (unsigned j{0}; j < rot_key_count; j++) {
          if (auto const& [timestamp, value]{frame_packet.anim_rot_keys[rot_key_begin_local_idx + j]};
            timestamp > cur_animation_time) {
            auto const& [prev_timestamp, prev_value]{frame_packet.anim_rot_keys[rot_key_begin_local_idx + j - 1]};
            rot = Slerp(prev_value, value, calc_interpolation_factor(prev_timestamp, timestamp, cur_animation_time));
            break;
          }
        }
      }

      if (scaling_key_count == 1) {
        scale = frame_packet.anim_scaling_keys[scaling_key_begin_local_idx].value;
      } else {
        for (unsigned j{0}; j < scaling_key_count; j++) {
          if (auto const& [timestamp, value]{frame_packet.anim_scaling_keys[scaling_key_begin_local_idx + j]};
            timestamp > cur_animation_time) {
            auto const& [prev_timestamp, prev_value]{
              frame_packet.anim_scaling_keys[scaling_key_begin_local_idx + j - 1]
            };
            scale = Lerp(prev_value, value, calc_interpolation_factor(prev_timestamp, timestamp, cur_animation_time));
            break;
          }
        }
      }

      frame_packet.skeleton_node_data[skeleton_begin_local_idx + node_idx].transform =
        Matrix4::Scale(scale) * static_cast<Matrix4>(rot) * Matrix4::Translate(pos);
    }

    // Accumulate node transforms

    for (unsigned i{0}; i < skeleton_size; i++) {
      if (auto& [transform, parent_idx]{frame_packet.skeleton_node_data[skeleton_begin_local_idx + i]}; parent_idx) {
        transform = transform * frame_packet.skeleton_node_data[skeleton_begin_local_idx + *parent_idx].transform;
      }
    }

    // Update bone matrices

    auto const bone_buf{frame_packet.buffers[bone_matrix_buf_local_idx]};
    for (unsigned i{0}; i < bone_count; i++) {
      // TODO this is horrible, update all bones at once
      auto const bone_mtx{
        frame_packet.bone_data[bone_begin_local_idx + i].offset_mtx * frame_packet.skeleton_node_data[
          skeleton_begin_local_idx + frame_packet.bone_data[bone_begin_local_idx + i].skeleton_node_idx].transform
      };
      render_manager_->UpdateBuffer(*bone_buf, static_cast<UINT>(i * sizeof(Matrix4)),
        as_bytes(std::span{&bone_mtx, 1}));
    }

    auto const& mesh_data{frame_packet.mesh_data[mesh_data_local_idx]};

    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, vtx_buf_idx),
      frame_packet.buffers[original_vertex_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, norm_buf_idx),
      frame_packet.buffers[original_normal_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, tan_buf_idx),
      frame_packet.buffers[original_tangent_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, bone_weight_buf_idx),
      frame_packet.buffers[bone_weight_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, bone_idx_buf_idx),
      frame_packet.buffers[bone_index_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, bone_buf_idx),
      frame_packet.buffers[bone_matrix_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, skinned_vtx_buf_idx),
      frame_packet.buffers[mesh_data.pos_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, skinned_norm_buf_idx),
      frame_packet.buffers[mesh_data.norm_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, skinned_tan_buf_idx),
      frame_packet.buffers[mesh_data.tan_buf_local_idx]->GetUnorderedAccess());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(VertexSkinningDrawParams, vtx_count), mesh_data.vtx_count);
    prepare_cmd.Dispatch(
      static_cast<UINT>(std::ceil(static_cast<float>(mesh_data.vtx_count) / static_cast<float>(SKINNING_CS_THREADS))),
      1, 1);
  }

  // Draw irradiance map
  if (frame_packet.draw_irradiance_map) {
    auto const cube_mesh{App::Instance().GetResourceManager().GetCubeMesh()};

    prepare_cmd.SetPipelineState(*frame_packet.irradiance_pso);
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(IrradianceDrawParams, pos_buf_idx),
      cube_mesh->GetPositionBuffer()->GetShaderResource());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(IrradianceDrawParams, cubemap_idx),
      frame_packet.skybox_cubemap->GetShaderResource());
    prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(IrradianceDrawParams, samp_idx), samp_af16_clamp_.Get());
    prepare_cmd.SetIndexBuffer(*cube_mesh->GetIndexBuffer(), cube_mesh->GetIndexFormat());

    prepare_cmd.Barrier({}, {}, std::array{
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
        D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
        frame_packet.irradiance_tex.get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      }
    });

    CD3DX12_VIEWPORT const irradiance_viewport{
      0.0f, 0.0f, static_cast<float>(frame_packet.irradiance_tex->GetDesc().width),
      static_cast<float>(frame_packet.irradiance_tex->GetDesc().height)
    };

    CD3DX12_RECT const irradiance_scissor{
      0, 0, static_cast<LONG>(frame_packet.irradiance_tex->GetDesc().width),
      static_cast<LONG>(frame_packet.irradiance_tex->GetDesc().height)
    };

    prepare_cmd.SetViewports(std::span{static_cast<D3D12_VIEWPORT const*>(&irradiance_viewport), 1});
    prepare_cmd.SetScissorRects(std::span{static_cast<D3D12_RECT const*>(&irradiance_scissor), 1});

    prepare_cmd.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    prepare_cmd.SetRenderTargets(std::span{frame_packet.irradiance_tex.get(), 1}, nullptr);
    prepare_cmd.ClearRenderTarget(*frame_packet.irradiance_tex, std::array{0.0f, 0.0f, 0.0f, 1.0f}, {});

    for (auto face_idx{0}; face_idx < 6; face_idx++) {
      auto const static global_cube_face_view_proj_matrices{
        [] {
          auto const proj_mtx{
            TransformProjectionMatrixForRendering(Matrix4::PerspectiveFov(ToRadians(90), 1, 0.1f, 2.0f))
          };

          auto matrices{MakeCubeFaceViewMatrices(Vector3::Zero())};

          for (auto& view_mtx : matrices) {
            view_mtx *= proj_mtx;
          }

          return matrices;
        }()
      };

      prepare_cmd.SetPipelineParameters(PIPELINE_PARAM_INDEX(IrradianceDrawParams, view_proj_mtx),
        std::span{std::bit_cast<UINT const*>(global_cube_face_view_proj_matrices[face_idx].GetData()), 16});
      prepare_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(IrradianceDrawParams, rt_array_idx),
        static_cast<UINT>(face_idx));

      prepare_cmd.DrawIndexedInstanced(
        static_cast<UINT>(App::Instance().GetResourceManager().GetCubeMesh()->GetIndexCount()),
        1, 0, 0, 0);
    }

    prepare_cmd.Barrier({}, {}, std::array{
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_ACCESS_RENDER_TARGET,
        D3D12_BARRIER_ACCESS_NO_ACCESS, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
        D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
        frame_packet.irradiance_tex.get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      }
    });
  }

  prepare_cmd.End();
  device_->ExecuteCommandLists(std::span{&prepare_cmd, 1});

  for (auto const& cam_data : frame_packet.cam_data) {
    auto& target_rt{*frame_packet.render_targets[cam_data.rt_local_idx]};
    auto const& target_rt_desc{target_rt.GetDesc()};

    auto const target_rt_width{target_rt_desc.width};
    auto const target_rt_height{target_rt_desc.height};

    CD3DX12_VIEWPORT const cam_viewport{
      cam_data.viewport.left * static_cast<FLOAT>(target_rt_width),
      cam_data.viewport.top * static_cast<float>(target_rt_height),
      std::max(
        cam_data.viewport.right * static_cast<float>(target_rt_width) - cam_data.viewport.left * static_cast<FLOAT>(
          target_rt_width), 1.0f),
      std::max(
        cam_data.viewport.bottom * static_cast<float>(target_rt_height) - cam_data.viewport.top * static_cast<float>(
          target_rt_height), 1.0f),
    };

    CD3DX12_RECT const cam_scissor{
      static_cast<LONG>(cam_data.viewport.left * static_cast<FLOAT>(target_rt_width)),
      static_cast<LONG>(cam_data.viewport.top * static_cast<FLOAT>(target_rt_height)),
      std::max(static_cast<LONG>(cam_data.viewport.right * static_cast<FLOAT>(target_rt_width)), 1l),
      std::max(static_cast<LONG>(cam_data.viewport.bottom * static_cast<FLOAT>(target_rt_height)), 1l),
    };

    auto const viewport_aspect{cam_viewport.Width / cam_viewport.Height};

    auto const transient_rt_width{static_cast<UINT>(cam_viewport.Width)};
    auto const transient_rt_height{static_cast<UINT>(cam_viewport.Height)};

    CD3DX12_VIEWPORT const transient_viewport{
      0.0f, 0.0f, static_cast<FLOAT>(transient_rt_width), static_cast<FLOAT>(transient_rt_height)
    };

    CD3DX12_RECT const transient_scissor{
      0, 0, static_cast<LONG>(transient_rt_width), static_cast<LONG>(transient_rt_height)
    };

    RenderTarget::Desc const hdr_rt_desc{
      transient_rt_width, transient_rt_height, frame_packet.color_buffer_format, depth_format_,
      static_cast<UINT>(frame_packet.msaa_mode), L"Camera HDR RenderTarget", false, frame_packet.background_color, 0.0f
    };

    auto& per_frame_cb{per_frame_cbs_[frame_idx]};
    SetPerFrameConstants(per_frame_cb, static_cast<int>(transient_rt_width), static_cast<int>(transient_rt_height),
      frame_packet.ambient_light, frame_packet.shadow_params);

    auto const cam_view_mtx{
      Camera::CalculateViewMatrix(cam_data.position, cam_data.right, cam_data.up, cam_data.forward)
    };
    auto const cam_proj_mtx{
      TransformProjectionMatrixForRendering(Camera::CalculateProjectionMatrix(cam_data.type, cam_data.fov_vert_deg,
        cam_data.size_vert, viewport_aspect, cam_data.near_plane, cam_data.far_plane))
    };
    auto const cam_view_proj_mtx{cam_view_mtx * cam_proj_mtx};
    Frustum const cam_frust_ws{cam_view_proj_mtx};

    std::pmr::vector<unsigned> visible_light_indices;
    CullLights(cam_frust_ws, frame_packet.light_data, visible_light_indices);

    // Performs rendering of the camera
    auto& cam_cmd{render_manager_->AcquireCommandList()};
    cam_cmd.Begin(nullptr);
    cam_cmd.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    // Shadow pass
    std::array<Matrix4, MAX_CASCADE_COUNT> shadow_view_proj_matrices;
    auto const shadow_cascade_boundaries{CalculateCameraShadowCascadeBoundaries(cam_data, frame_packet.shadow_params)};
    DrawDirectionalShadowMaps(frame_packet, visible_light_indices, cam_data, viewport_aspect,
      frame_packet.shadow_params.cascade_count, shadow_cascade_boundaries, shadow_view_proj_matrices, cam_cmd);

    UpdatePunctualShadowAtlas(*punctual_shadow_atlas_, frame_packet.light_data, visible_light_indices, cam_data,
      cam_view_proj_mtx, frame_packet.shadow_params.distance);
    DrawPunctualShadowMaps(*punctual_shadow_atlas_, frame_packet, cam_cmd);

    std::pmr::vector<unsigned> visible_static_submesh_instance_indices;
    CullStaticSubmeshInstances(cam_frust_ws, frame_packet.mesh_data, frame_packet.submesh_data,
      frame_packet.instance_data, visible_static_submesh_instance_indices);

    auto& cam_per_view_cb{AcquirePerViewConstantBuffer()};
    SetPerViewConstants(cam_per_view_cb, cam_view_mtx, cam_proj_mtx, shadow_cascade_boundaries, cam_data.position);

    auto const hdr_rt{render_manager_->AcquireTemporaryRenderTarget(hdr_rt_desc)};

    cam_cmd.Barrier({}, {}, std::array{
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_ACCESS_NO_ACCESS,
        D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE, D3D12_BARRIER_LAYOUT_UNDEFINED,
        D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE, hdr_rt->GetDepthStencilTex().get(), {0, 1, 0, 1, 0, 1},
        D3D12_TEXTURE_BARRIER_FLAG_NONE
      },
    });

    cam_cmd.ClearDepthStencil(*hdr_rt->GetDepthStencilTex(), D3D12_CLEAR_FLAG_DEPTH, 0, 0, {});

    cam_cmd.SetViewports(std::span{static_cast<D3D12_VIEWPORT const*>(&transient_viewport), 1});
    cam_cmd.SetScissorRects(std::span{static_cast<D3D12_RECT const*>(&transient_scissor), 1});

    auto const normal_rt{
      render_manager_->AcquireTemporaryRenderTarget(RenderTarget::Desc{
        transient_rt_width, transient_rt_height, normal_buffer_format_, std::nullopt, 1, L"Camera Normal RT"
      })
    };

    // Depth-Normal pre-pass
    if (frame_packet.depth_normal_pre_pass_enabled) {
      // If MSAA is enabled we render into an MSAA RT and then resolve into normalRt
      auto const actual_normal_rt{
        [this, &normal_rt, &frame_packet] {
          if (frame_packet.msaa_mode == MultisamplingMode::kOff) {
            return normal_rt;
          }

          auto actual_normal_rt_desc{normal_rt->GetDesc()};
          actual_normal_rt_desc.sample_count = static_cast<int>(frame_packet.msaa_mode);
          return render_manager_->AcquireTemporaryRenderTarget(actual_normal_rt_desc);
        }()
      };

      cam_cmd.SetPipelineState(*frame_packet.depth_normal_pso);
      cam_cmd.SetRenderTargets(std::span{actual_normal_rt->GetColorTex().get(), 1}, hdr_rt->GetDepthStencilTex().get());
      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
          D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          actual_normal_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });
      cam_cmd.ClearRenderTarget(*actual_normal_rt->GetColorTex(), std::array{0.0f, 0.0f, 0.0f, 1.0f}, {});

      for (auto const instance_idx : visible_static_submesh_instance_indices) {
        auto const& instance{frame_packet.instance_data[instance_idx]};
        auto const& submesh{frame_packet.submesh_data[instance.submesh_local_idx]};
        auto const& mesh{frame_packet.mesh_data[submesh.mesh_local_idx]};
        auto const& mtl_buf{frame_packet.buffers[submesh.mtl_buf_local_idx]};

        auto& per_draw_cb{AcquirePerDrawConstantBuffer()};
        SetPerDrawConstants(per_draw_cb, instance.local_to_world_mtx);

        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, pos_buf_idx),
          frame_packet.buffers[mesh.pos_buf_local_idx]->GetShaderResource());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, norm_buf_idx),
          frame_packet.buffers[mesh.norm_buf_local_idx]->GetShaderResource());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, tan_buf_idx),
          frame_packet.buffers[mesh.tan_buf_local_idx]->GetShaderResource());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, uv_buf_idx),
          frame_packet.buffers[mesh.uv_buf_local_idx]->GetShaderResource());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, mtl_idx),
          mtl_buf->GetConstantBuffer());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, samp_idx), samp_af16_wrap_.Get());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, rt_idx), 0);
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, per_draw_cb_idx),
          per_draw_cb.GetBuffer()->GetConstantBuffer());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, per_view_cb_idx),
          cam_per_view_cb.GetBuffer()->GetConstantBuffer());
        cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthNormalDrawParams, per_frame_cb_idx),
          per_frame_cb.GetBuffer()->GetConstantBuffer());
        cam_cmd.SetIndexBuffer(*frame_packet.buffers[mesh.idx_buf_local_idx], mesh.idx_format);
        cam_cmd.DrawIndexedInstanced(submesh.index_count, 1, submesh.first_index, submesh.base_vertex, 0);
      }

      // If we have MSAA enabled, actualNormalRt is an MSAA texture that we have to resolve into normalRt
      if (frame_packet.msaa_mode != MultisamplingMode::kOff) {
        cam_cmd.Barrier({}, {}, std::array{
          graphics::TextureBarrier{
            D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_ACCESS_RENDER_TARGET,
            D3D12_BARRIER_ACCESS_RESOLVE_SOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
            D3D12_BARRIER_LAYOUT_RESOLVE_SOURCE, actual_normal_rt->GetColorTex().get(), {0, 0},
            D3D12_TEXTURE_BARRIER_FLAG_NONE
          },
          graphics::TextureBarrier{
            D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_ACCESS_NO_ACCESS,
            D3D12_BARRIER_ACCESS_RESOLVE_DEST, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RESOLVE_DEST,
            normal_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
          }
        });
        cam_cmd.Resolve(*normal_rt->GetColorTex(), *actual_normal_rt->GetColorTex(),
          *normal_rt->GetDesc().color_format);
        cam_cmd.Barrier({}, {}, std::array{
          graphics::TextureBarrier{
            D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RESOLVE_DEST,
            D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RESOLVE_DEST,
            D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, normal_rt->GetColorTex().get(), {0, 0},
            D3D12_TEXTURE_BARRIER_FLAG_NONE
          }
        });
      } else {
        cam_cmd.Barrier({}, {}, std::array{
          graphics::TextureBarrier{
            D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RENDER_TARGET,
            D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
            D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, actual_normal_rt->GetColorTex().get(), {0, 0},
            D3D12_TEXTURE_BARRIER_FLAG_NONE
          }
        });
      }
    }

    auto ssao_tex{white_tex_.get()};

    // SSAO pass
    if (frame_packet.ssao_enabled) {
      auto const ssao_rt{
        render_manager_->AcquireTemporaryRenderTarget(RenderTarget::Desc{
          transient_rt_width, transient_rt_height, ssao_buffer_format_, std::nullopt, 1, L"SSAO RT"
        })
      };

      // If MSAA is enabled we have to resolve the depth texture before running SSAO
      auto const ssao_depth_tex{
        [this, &hdr_rt, &cam_cmd, &frame_packet] {
          if (frame_packet.msaa_mode == MultisamplingMode::kOff) {
            cam_cmd.Barrier({}, {}, std::array{
              graphics::TextureBarrier{
                D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_SYNC_PIXEL_SHADING,
                D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE, D3D12_BARRIER_ACCESS_SHADER_RESOURCE,
                D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE, D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
                hdr_rt->GetDepthStencilTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
              }
            });
            return hdr_rt->GetDepthStencilTex();
          }

          auto ssao_depth_rt_desc{hdr_rt->GetDesc()};
          ssao_depth_rt_desc.color_format = DXGI_FORMAT_R32_FLOAT;
          ssao_depth_rt_desc.sample_count = 1;
          ssao_depth_rt_desc.depth_stencil_format = std::nullopt;
          ssao_depth_rt_desc.enable_unordered_access = true;
          auto const ssao_depth_rt{render_manager_->AcquireTemporaryRenderTarget(ssao_depth_rt_desc)};

          cam_cmd.Barrier({}, {}, std::array{
            graphics::TextureBarrier{
              D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_SYNC_COMPUTE_SHADING,
              D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE, D3D12_BARRIER_ACCESS_SHADER_RESOURCE,
              D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE, D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
              hdr_rt->GetDepthStencilTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
            },
            graphics::TextureBarrier{
              D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_COMPUTE_SHADING, D3D12_BARRIER_ACCESS_NO_ACCESS,
              D3D12_BARRIER_ACCESS_UNORDERED_ACCESS, D3D12_BARRIER_LAYOUT_UNDEFINED,
              D3D12_BARRIER_LAYOUT_UNORDERED_ACCESS, ssao_depth_rt->GetColorTex().get(), {0, 0},
              D3D12_TEXTURE_BARRIER_FLAG_NONE
            }
          });

          cam_cmd.SetPipelineState(*frame_packet.depth_resolve_pso);
          cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthResolveDrawParams, in_tex_idx),
            hdr_rt->GetDepthStencilTex()->GetShaderResource());
          cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(DepthResolveDrawParams, out_tex_idx),
            ssao_depth_rt->GetColorTex()->GetUnorderedAccess());

          cam_cmd.Dispatch(
            static_cast<UINT>(std::ceil(static_cast<float>(ssao_depth_rt_desc.width) / DEPTH_RESOLVE_CS_THREADS_X)),
            static_cast<UINT>(std::ceil(static_cast<float>(ssao_depth_rt_desc.height) / DEPTH_RESOLVE_CS_THREADS_Y)),
            static_cast<UINT>(std::ceil(1.0f / DEPTH_RESOLVE_CS_THREADS_Z)));

          cam_cmd.Barrier({}, {}, std::array{
            graphics::TextureBarrier{
              D3D12_BARRIER_SYNC_COMPUTE_SHADING, D3D12_BARRIER_SYNC_PIXEL_SHADING,
              D3D12_BARRIER_ACCESS_UNORDERED_ACCESS, D3D12_BARRIER_ACCESS_SHADER_RESOURCE,
              D3D12_BARRIER_LAYOUT_UNORDERED_ACCESS, D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
              ssao_depth_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
            }
          });

          return ssao_depth_rt->GetColorTex();
        }()
      };

      cam_cmd.SetPipelineState(*frame_packet.ssao_pso);
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, noise_tex_idx),
        ssao_noise_tex_->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, depth_tex_idx),
        ssao_depth_tex->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, normal_tex_idx),
        normal_rt->GetColorTex()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, samp_buf_idx),
        ssao_samples_buffer_.GetBuffer()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, point_clamp_samp_idx), samp_point_clamp_.Get());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, point_wrap_samp_idx), samp_point_wrap_.Get());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, radius),
        *std::bit_cast<UINT*>(&frame_packet.ssao_params.radius));
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, bias),
        *std::bit_cast<UINT*>(&frame_packet.ssao_params.bias));
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, power),
        *std::bit_cast<UINT*>(&frame_packet.ssao_params.power));
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, sample_count),
        frame_packet.ssao_params.sample_count);
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, per_view_cb_idx),
        cam_per_view_cb.GetBuffer()->GetConstantBuffer());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoDrawParams, per_frame_cb_idx),
        per_frame_cb.GetBuffer()->GetConstantBuffer());
      cam_cmd.SetRenderTargets(std::span{ssao_rt->GetColorTex().get(), 1}, nullptr);
      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
          D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          ssao_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });
      cam_cmd.ClearRenderTarget(*ssao_rt->GetColorTex(), std::array{0.0f, 0.0f, 0.0f, 1.0f}, {});
      cam_cmd.DrawInstanced(3, 1, 0, 0);

      auto const ssao_blur_rt{
        render_manager_->AcquireTemporaryRenderTarget([&ssao_rt] {
          auto ret{ssao_rt->GetDesc()};
          ret.debug_name = L"SSAO Blur RT";
          return ret;
        }())
      };

      cam_cmd.SetPipelineState(*frame_packet.ssao_blur_pso);
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoBlurDrawParams, in_tex_idx),
        ssao_rt->GetColorTex()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SsaoBlurDrawParams, point_clamp_samp_idx),
        samp_point_clamp_.Get());
      cam_cmd.SetRenderTargets(std::span{ssao_blur_rt->GetColorTex().get(), 1}, nullptr);
      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
          D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          ssao_blur_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        },
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RENDER_TARGET,
          D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, ssao_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });
      cam_cmd.ClearRenderTarget(*ssao_blur_rt->GetColorTex(), std::array{0.0f, 0.0f, 0.0f, 1.0f}, {});
      cam_cmd.DrawInstanced(3, 1, 0, 0);
      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RENDER_TARGET,
          D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, ssao_blur_rt->GetColorTex().get(), {0, 0},
          D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });

      ssao_tex = ssao_blur_rt->GetColorTex().get();
    }

    // Full forward lighting pass

    auto const light_count{std::ssize(visible_light_indices)};
    auto& light_buffer{light_buffers_[frame_idx]};
    light_buffer.Resize(static_cast<int>(light_count));
    auto const light_buffer_data{light_buffer.GetData()};

    for (auto i = 0; i < light_count; i++) {
      light_buffer_data[i].color = frame_packet.light_data[visible_light_indices[i]].color;
      light_buffer_data[i].intensity = frame_packet.light_data[visible_light_indices[i]].intensity;
      light_buffer_data[i].type = static_cast<int>(frame_packet.light_data[visible_light_indices[i]].type);
      light_buffer_data[i].direction = frame_packet.light_data[visible_light_indices[i]].direction;
      light_buffer_data[i].isCastingShadow = FALSE;
      light_buffer_data[i].range = frame_packet.light_data[visible_light_indices[i]].range;
      light_buffer_data[i].halfInnerAngleCos = std::cos(
        ToRadians(frame_packet.light_data[visible_light_indices[i]].inner_angle / 2.0f));
      light_buffer_data[i].halfOuterAngleCos = std::cos(
        ToRadians(frame_packet.light_data[visible_light_indices[i]].outer_angle / 2.0f));
      light_buffer_data[i].position = frame_packet.light_data[visible_light_indices[i]].position;
      light_buffer_data[i].depthBias = frame_packet.light_data[visible_light_indices[i]].shadow_depth_bias;
      light_buffer_data[i].normalBias = frame_packet.light_data[visible_light_indices[i]].shadow_normal_bias;

      for (auto& sample : light_buffer_data[i].sampleShadowMap) {
        sample = FALSE;
      }
    }

    // Set directional light shadow constant data

    for (auto i{0}; i < light_count; i++) {
      if (auto const light{frame_packet.light_data[visible_light_indices[i]]};
        light.type == LightComponent::Type::Directional && light.casts_shadow) {
        light_buffer_data[i].isCastingShadow = TRUE;

        for (auto cascade_idx{0}; cascade_idx < frame_packet.shadow_params.cascade_count; cascade_idx++) {
          light_buffer_data[i].sampleShadowMap[cascade_idx] = TRUE;
          light_buffer_data[i].shadowViewProjMatrices[cascade_idx] = shadow_view_proj_matrices[cascade_idx];
        }

        break;
      }
    }

    punctual_shadow_atlas_->SetLookUpInfo(light_buffer_data);

    auto const forward_pass_depth_tex_sync_before{
      frame_packet.ssao_enabled
        ? frame_packet.msaa_mode != MultisamplingMode::kOff
            ? D3D12_BARRIER_SYNC_COMPUTE_SHADING
            : D3D12_BARRIER_SYNC_PIXEL_SHADING
        : D3D12_BARRIER_SYNC_DEPTH_STENCIL
    };

    auto const forward_pass_depth_tex_access_before{
      frame_packet.ssao_enabled ? D3D12_BARRIER_ACCESS_SHADER_RESOURCE : D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE
    };

    auto const forward_pass_depth_tex_access_after{
      frame_packet.depth_normal_pre_pass_enabled
        ? D3D12_BARRIER_ACCESS_DEPTH_STENCIL_READ
        : D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE
    };

    auto const forward_pass_depth_tex_layout_before{
      frame_packet.ssao_enabled ? D3D12_BARRIER_LAYOUT_SHADER_RESOURCE : D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE
    };

    auto const forward_pass_depth_tex_layout_after{
      frame_packet.depth_normal_pre_pass_enabled
        ? D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_READ
        : D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE
    };

    cam_cmd.Barrier({}, {}, std::array{
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE,
        D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE,
        D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, dir_shadow_map_arr_->GetTex().get(), {0xffffffff, 0},
        D3D12_TEXTURE_BARRIER_FLAG_NONE
      },
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_DEPTH_STENCIL, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_DEPTH_STENCIL_WRITE,
        D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_DEPTH_STENCIL_WRITE,
        D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, punctual_shadow_atlas_->GetTex().get(), {0, 0},
        D3D12_TEXTURE_BARRIER_FLAG_NONE
      },
      graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_ACCESS_NO_ACCESS,
        D3D12_BARRIER_ACCESS_RENDER_TARGET, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
        hdr_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      },
      graphics::TextureBarrier{
        forward_pass_depth_tex_sync_before, D3D12_BARRIER_SYNC_DEPTH_STENCIL, forward_pass_depth_tex_access_before,
        forward_pass_depth_tex_access_after, forward_pass_depth_tex_layout_before, forward_pass_depth_tex_layout_after,
        hdr_rt->GetDepthStencilTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      }
    });

    cam_cmd.SetPipelineState(frame_packet.depth_normal_pre_pass_enabled
                               ? *frame_packet.object_pso_depth_read
                               : *frame_packet.object_pso_depth_write);
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, mtl_samp_idx), samp_af16_wrap_.Get());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, point_clamp_samp_idx), samp_point_clamp_.Get());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, shadow_samp_idx), samp_cmp_pcf_ge_.Get());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, ssao_tex_idx), ssao_tex->GetShaderResource());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, light_buf_idx),
      light_buffer.GetBuffer()->GetShaderResource());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, light_count), static_cast<UINT>(light_count));
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, dir_shadow_arr_idx),
      dir_shadow_map_arr_->GetTex()->GetShaderResource());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, punc_shadow_atlas_idx),
      punctual_shadow_atlas_->GetTex()->GetShaderResource());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, per_view_cb_idx),
      cam_per_view_cb.GetBuffer()->GetConstantBuffer());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, per_frame_cb_idx),
      per_frame_cb.GetBuffer()->GetConstantBuffer());
    cam_cmd.SetRenderTargets(std::span{hdr_rt->GetColorTex().get(), 1}, hdr_rt->GetDepthStencilTex().get());
    cam_cmd.ClearRenderTarget(*hdr_rt->GetColorTex(), frame_packet.background_color, {});

    for (auto const instance_idx : visible_static_submesh_instance_indices) {
      auto const& instance{frame_packet.instance_data[instance_idx]};
      auto const& submesh{frame_packet.submesh_data[instance.submesh_local_idx]};
      auto const& mesh{frame_packet.mesh_data[submesh.mesh_local_idx]};
      auto const& mtl_buf{frame_packet.buffers[submesh.mtl_buf_local_idx]};

      auto& per_draw_cb{AcquirePerDrawConstantBuffer()};
      SetPerDrawConstants(per_draw_cb, instance.local_to_world_mtx);

      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, pos_buf_idx),
        frame_packet.buffers[mesh.pos_buf_local_idx]->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, norm_buf_idx),
        frame_packet.buffers[mesh.norm_buf_local_idx]->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, tan_buf_idx),
        frame_packet.buffers[mesh.tan_buf_local_idx]->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, uv_buf_idx),
        frame_packet.buffers[mesh.uv_buf_local_idx]->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, mtl_idx), mtl_buf->GetConstantBuffer());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(ObjectDrawParams, per_draw_cb_idx),
        per_draw_cb.GetBuffer()->GetConstantBuffer());
      cam_cmd.SetIndexBuffer(*frame_packet.buffers[mesh.idx_buf_local_idx], mesh.idx_format);
      cam_cmd.DrawIndexedInstanced(submesh.index_count, 1, submesh.first_index, submesh.base_vertex, 0);
    }

    if (frame_packet.skybox_cubemap) {
      auto const cube_mesh{App::Instance().GetResourceManager().GetCubeMesh()};
      cam_cmd.SetPipelineState(*frame_packet.skybox_pso);
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SkyboxDrawParams, pos_buf_idx),
        cube_mesh->GetPositionBuffer()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SkyboxDrawParams, per_view_cb_idx),
        cam_per_view_cb.GetBuffer()->GetConstantBuffer());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SkyboxDrawParams, cubemap_idx),
        //frame_packet.skybox_cubemap->GetShaderResource());
        frame_packet.irradiance_tex->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(SkyboxDrawParams, samp_idx), samp_af16_clamp_.Get());
      cam_cmd.SetIndexBuffer(*cube_mesh->GetIndexBuffer(), cube_mesh->GetIndexFormat());
      cam_cmd.DrawIndexedInstanced(
        static_cast<UINT>(App::Instance().GetResourceManager().GetCubeMesh()->GetIndexCount()),
        1, 0, 0, 0);
    }

    RenderTarget const* post_process_input_rt;

    if (frame_packet.msaa_mode == MultisamplingMode::kOff) {
      post_process_input_rt = hdr_rt.get();
      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RENDER_TARGET,
          D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET,
          D3D12_BARRIER_LAYOUT_SHADER_RESOURCE, hdr_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });
    } else {
      auto resolved_hdr_rt_desc{hdr_rt_desc};
      resolved_hdr_rt_desc.sample_count = 1;
      auto const resolve_hdr_rt{render_manager_->AcquireTemporaryRenderTarget(resolved_hdr_rt_desc)};

      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_RENDER_TARGET, D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_ACCESS_RENDER_TARGET,
          D3D12_BARRIER_ACCESS_RESOLVE_SOURCE, D3D12_BARRIER_LAYOUT_RENDER_TARGET, D3D12_BARRIER_LAYOUT_RESOLVE_SOURCE,
          hdr_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        },
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_ACCESS_NO_ACCESS,
          D3D12_BARRIER_ACCESS_RESOLVE_DEST, D3D12_BARRIER_LAYOUT_UNDEFINED, D3D12_BARRIER_LAYOUT_RESOLVE_DEST,
          resolve_hdr_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });

      cam_cmd.Resolve(*resolve_hdr_rt->GetColorTex(), *hdr_rt->GetColorTex(), *hdr_rt_desc.color_format);

      cam_cmd.Barrier({}, {}, std::array{
        graphics::TextureBarrier{
          D3D12_BARRIER_SYNC_RESOLVE, D3D12_BARRIER_SYNC_PIXEL_SHADING, D3D12_BARRIER_ACCESS_RESOLVE_DEST,
          D3D12_BARRIER_ACCESS_SHADER_RESOURCE, D3D12_BARRIER_LAYOUT_RESOLVE_DEST, D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
          resolve_hdr_rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
        }
      });

      post_process_input_rt = resolve_hdr_rt.get();
    }

    cam_cmd.SetViewports(std::span{static_cast<D3D12_VIEWPORT const*>(&cam_viewport), 1});
    cam_cmd.SetScissorRects(std::span{static_cast<D3D12_RECT const*>(&cam_scissor), 1});

    cam_cmd.SetPipelineState(*frame_packet.post_process_pso);
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(PostProcessDrawParams, in_tex_idx),
      post_process_input_rt->GetColorTex()->GetShaderResource());
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(PostProcessDrawParams, inv_gamma),
      *std::bit_cast<UINT*>(&frame_packet.inv_gamma));
    cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(PostProcessDrawParams, bi_clamp_samp_idx), samp_bi_clamp_.Get());
    cam_cmd.SetRenderTargets(std::span{target_rt.GetColorTex().get(), 1}, nullptr);
    cam_cmd.DrawInstanced(3, 1, 0, 0);

    if (!frame_packet.line_gizmo_vertex_data.empty()) {
      cam_cmd.SetPipelineState(*frame_packet.line_gizmo_pso);
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(GizmoDrawParams, vertex_buf_idx),
        line_gizmo_vertex_data_buffer_.GetBuffer()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(GizmoDrawParams, color_buf_idx),
        gizmo_color_buffer_.GetBuffer()->GetShaderResource());
      cam_cmd.SetPipelineParameter(PIPELINE_PARAM_INDEX(GizmoDrawParams, per_view_cb_idx),
        cam_per_view_cb.GetBuffer()->GetConstantBuffer());
      cam_cmd.SetRenderTargets(std::span{target_rt.GetColorTex().get(), 1}, nullptr);
      cam_cmd.SetScissorRects(std::span{static_cast<D3D12_RECT const*>(&cam_scissor), 1});
      cam_cmd.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
      cam_cmd.DrawInstanced(2, static_cast<UINT>(frame_packet.line_gizmo_vertex_data.size()), 0, 0);
    }

    cam_cmd.End();
    device_->ExecuteCommandLists(std::span{&cam_cmd, 1});
  }

  cam_rt_barriers.clear();

  std::ranges::transform(frame_packet.render_targets, std::back_inserter(cam_rt_barriers),
    [](std::shared_ptr<RenderTarget> const& rt) {
      return graphics::TextureBarrier{
        D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_SYNC_NONE, D3D12_BARRIER_ACCESS_NO_ACCESS,
        D3D12_BARRIER_ACCESS_NO_ACCESS, D3D12_BARRIER_LAYOUT_RENDER_TARGET, D3D12_BARRIER_LAYOUT_SHADER_RESOURCE,
        rt->GetColorTex().get(), {0, 0}, D3D12_TEXTURE_BARRIER_FLAG_NONE
      };
    });

  // Transitions the render targets back to shader resource state
  auto& finish_cmd{render_manager_->AcquireCommandList()};
  finish_cmd.Begin(nullptr);
  finish_cmd.Barrier({}, {}, cam_rt_barriers);
  finish_cmd.End();
  device_->ExecuteCommandLists(std::span{&finish_cmd, 1});
}


auto SceneRenderer::DrawLineAtNextRender(Vector3 const& from, Vector3 const& to, Color const& color) -> void {
  gizmo_colors_.emplace_back(color);
  line_gizmo_vertex_data_.emplace_back(from, static_cast<std::uint32_t>(gizmo_colors_.size() - 1), to, 0.0f);
}


auto SceneRenderer::GetRenderTargetOverride() -> std::shared_ptr<RenderTarget> const& {
  return rt_override_;
}


auto SceneRenderer::SetRenderTargetOverride(std::shared_ptr<RenderTarget> rt_override) -> void {
  rt_override_ = std::move(rt_override);
}


auto SceneRenderer::GetCurrentRenderTarget() const -> RenderTarget const& {
  return rt_override_ ? *rt_override_ : *main_rt_;
}


auto SceneRenderer::GetMultisamplingMode() const noexcept -> MultisamplingMode {
  return msaa_mode_;
}


auto SceneRenderer::SetMultisamplingMode(MultisamplingMode const mode) noexcept -> void {
  msaa_mode_ = mode;
  RecreatePipelines();
}


auto SceneRenderer::IsDepthNormalPrePassEnabled() const noexcept -> bool {
  return depth_normal_pre_pass_enabled_;
}


auto SceneRenderer::SetDepthNormalPrePassEnabled(bool const enabled) noexcept -> void {
  depth_normal_pre_pass_enabled_ = enabled;

  if (!enabled && IsSsaoEnabled()) {
    SetSsaoEnabled(false);
  }
}


auto SceneRenderer::IsUsingPreciseColorFormat() const noexcept -> bool {
  return color_buffer_format_ == precise_color_buffer_format_;
}


auto SceneRenderer::SetUsePreciseColorFormat(bool const precise) noexcept -> void {
  color_buffer_format_ = precise ? precise_color_buffer_format_ : imprecise_color_buffer_format_;
  RecreatePipelines();
}


auto SceneRenderer::GetShadowDistance() const noexcept -> float {
  return shadow_params_.distance;
}


auto SceneRenderer::SetShadowDistance(float const distance) noexcept -> void {
  shadow_params_.distance = std::max(0.0f, distance);
}


auto SceneRenderer::GetShadowCascadeCount() const noexcept -> int {
  return shadow_params_.cascade_count;
}


auto SceneRenderer::SetShadowCascadeCount(int const cascade_count) noexcept -> void {
  shadow_params_.cascade_count = std::clamp(cascade_count, 1, MAX_CASCADE_COUNT);
  int const splitCount{shadow_params_.cascade_count - 1};

  for (auto i = 1; i < splitCount; i++) {
    shadow_params_.normalized_cascade_splits[i] = std::max(shadow_params_.normalized_cascade_splits[i - 1],
      shadow_params_.normalized_cascade_splits[i]);
  }
}


auto SceneRenderer::GetNormalizedShadowCascadeSplits() const noexcept -> std::span<float const> {
  return {
    std::begin(shadow_params_.normalized_cascade_splits), static_cast<std::size_t>(shadow_params_.cascade_count - 1)
  };
}


auto SceneRenderer::SetNormalizedShadowCascadeSplit(int const idx, float const split) noexcept -> void {
  auto const splitCount{shadow_params_.cascade_count - 1};

  if (idx < 0 || idx >= splitCount) {
    return;
  }

  float const clampMin{idx == 0 ? 0.0f : shadow_params_.normalized_cascade_splits[idx - 1]};
  float const clampMax{idx == splitCount - 1 ? 1.0f : shadow_params_.normalized_cascade_splits[idx + 1]};

  shadow_params_.normalized_cascade_splits[idx] = std::clamp(split, clampMin, clampMax);
}


auto SceneRenderer::IsVisualizingShadowCascades() const noexcept -> bool {
  return shadow_params_.visualize_cascades;
}


auto SceneRenderer::VisualizeShadowCascades(bool const visualize) noexcept -> void {
  shadow_params_.visualize_cascades = visualize;
}


auto SceneRenderer::GetShadowFilteringMode() const noexcept -> ShadowFilteringMode {
  return shadow_params_.filtering_mode;
}


auto SceneRenderer::SetShadowFilteringMode(ShadowFilteringMode const filtering_mode) noexcept -> void {
  shadow_params_.filtering_mode = filtering_mode;
}


auto SceneRenderer::IsSsaoEnabled() const noexcept -> bool {
  return ssao_enabled_;
}


auto SceneRenderer::SetSsaoEnabled(bool const enabled) noexcept -> void {
  ssao_enabled_ = enabled;

  if (enabled && !IsDepthNormalPrePassEnabled()) {
    SetDepthNormalPrePassEnabled(true);
  }
}


auto SceneRenderer::GetSsaoParams() const noexcept -> SsaoParams const& {
  return ssao_params_;
}


auto SceneRenderer::SetSsaoParams(SsaoParams const& ssao_params) noexcept -> void {
  if (ssao_params_.sample_count != ssao_params.sample_count) {
    RecreateSsaoSamples(ssao_params.sample_count);
  }

  ssao_params_ = ssao_params;
}


auto SceneRenderer::GetGamma() const noexcept -> f32 {
  return 1.f / inv_gamma_;
}


auto SceneRenderer::SetGamma(f32 const gamma) noexcept -> void {
  inv_gamma_ = 1.f / gamma;
}


auto SceneRenderer::Register(StaticMeshComponent const& static_mesh_component) noexcept -> void {
  static_mesh_components_.emplace_back(std::addressof(static_mesh_component));
}


auto SceneRenderer::Unregister(StaticMeshComponent const& static_mesh_component) noexcept -> void {
  std::erase(static_mesh_components_, std::addressof(static_mesh_component));
}


auto SceneRenderer::Register(SkinnedMeshComponent const& skinned_mesh_component) noexcept -> void {
  skinned_mesh_components_.emplace_back(std::addressof(skinned_mesh_component));
}


auto SceneRenderer::Unregister(SkinnedMeshComponent const& skinned_mesh_component) noexcept -> void {
  std::erase(skinned_mesh_components_, std::addressof(skinned_mesh_component));
}


auto SceneRenderer::Register(LightComponent const& light_component) noexcept -> void {
  lights_.emplace_back(std::addressof(light_component));
}


auto SceneRenderer::Unregister(LightComponent const& light_component) noexcept -> void {
  std::erase(lights_, std::addressof(light_component));
}


auto SceneRenderer::Register(Camera const& cam) noexcept -> void {
  cameras_.emplace_back(&cam);
}


auto SceneRenderer::Unregister(Camera const& cam) noexcept -> void {
  std::erase(cameras_, &cam);
}
}

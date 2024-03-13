#pragma once

#include "graphics.hpp"
#include "render_target.hpp"
#include "../Core.hpp"
#include "../Math.hpp"
#include "../observer_ptr.hpp"
#include "../resources/Material.hpp"
#include "../resources/Mesh.hpp"

#include <DirectXTex.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>


namespace sorcery::rendering {
class RenderManager {
public:
  LEOPPHAPI explicit RenderManager(graphics::GraphicsDevice& device);
  RenderManager(RenderManager const&) = delete;
  RenderManager(RenderManager&&) = delete;

  ~RenderManager() = default;

  auto operator=(RenderManager const&) -> void = delete;
  auto operator=(RenderManager&&) -> void = delete;

  LEOPPHAPI auto BeginNewFrame() -> void;

  [[nodiscard]] constexpr static auto GetMaxFramesInFlight() -> UINT;
  [[nodiscard]] LEOPPHAPI auto GetCurrentFrameIndex() const -> UINT;

  [[nodiscard]] LEOPPHAPI auto AcquireCommandList() -> graphics::CommandList&;
  [[nodiscard]] LEOPPHAPI auto
  GetTemporaryRenderTarget(RenderTarget::Desc const& desc) -> std::shared_ptr<RenderTarget>;

  [[nodiscard]] LEOPPHAPI auto LoadReadonlyTexture(
    DirectX::ScratchImage const& img) -> graphics::SharedDeviceChildHandle<graphics::Texture>;
  [[nodiscard]] LEOPPHAPI auto UpdateBuffer(graphics::Buffer const& buf, std::span<std::byte const> data) -> bool;

  [[nodiscard]] LEOPPHAPI auto GetDefaultMaterial() const noexcept -> ObserverPtr<Material>;
  [[nodiscard]] LEOPPHAPI auto GetCubeMesh() const noexcept -> ObserverPtr<Mesh>;
  [[nodiscard]] LEOPPHAPI auto GetPlaneMesh() const noexcept -> ObserverPtr<Mesh>;
  [[nodiscard]] LEOPPHAPI auto GetSphereMesh() const noexcept -> ObserverPtr<Mesh>;

private:
  struct TempRenderTargetRecord {
    std::shared_ptr<RenderTarget> rt;
    int age_in_frames;
  };


  auto CreateCommandLists(UINT count) -> void;
  auto ReleaseTempRenderTargets() noexcept -> void;

  static UINT constexpr max_tmp_rt_age_{10};
  static UINT constexpr max_frames_in_flight_{2};
  inline static Guid const default_material_guid_{1, 0};
  inline static Guid const cube_mesh_guid_{2, 0};
  inline static Guid const plane_mesh_guid_{3, 0};
  inline static Guid const sphere_mesh_guid_{4, 0};

  ObserverPtr<graphics::GraphicsDevice> device_;

  UINT frame_idx_{0};
  UINT next_cmd_list_idx_{0};

  std::vector<std::array<graphics::SharedDeviceChildHandle<graphics::CommandList>, max_frames_in_flight_>> cmd_lists_;
  std::vector<TempRenderTargetRecord> tmp_render_targets_;

  ObserverPtr<Material> default_mtl_;
  ObserverPtr<Mesh> cube_mesh_;
  ObserverPtr<Mesh> plane_mesh_;
  ObserverPtr<Mesh> sphere_mesh_;
};


constexpr auto RenderManager::GetMaxFramesInFlight() -> UINT {
  return max_frames_in_flight_;
}


auto TransformProjectionMatrixForRendering(Matrix4 const& proj_mtx) noexcept -> Matrix4;

auto GenerateSphereMesh(float radius, int latitudes, int longitudes, std::vector<Vector3>& vertices,
                        std::vector<Vector3>& normals, std::vector<Vector2>& uvs,
                        std::vector<std::uint32_t>& indices) -> void;
}

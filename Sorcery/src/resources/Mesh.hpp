#pragma once

#include "Resource.hpp"
#include "../Bounds.hpp"
#include "../Math.hpp"
#include "../rendering/graphics.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>


namespace sorcery {
class Mesh final : public Resource {
  RTTR_ENABLE(Resource)
  struct GeometryData {
    std::vector<Vector3> positions;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<Vector3> tangents;
    std::vector<std::uint16_t> indices16;
    std::vector<std::uint32_t> indices32;
  };

public:
  struct MeshletData {
    std::uint32_t vert_count;
    std::uint32_t vert_offset;
    std::uint32_t prim_count;
    std::uint32_t prim_offset;
  };


  struct MeshletTriangleIndexData {
    std::uint32_t idx0 : 10;
    std::uint32_t idx1 : 10;
    std::uint32_t idx2 : 10;
  };


  struct MaterialSlotInfo {
    std::string name;
  };


  struct SubMeshInfo {
    int base_vertex;
    int first_index;
    int index_count;
    int material_index;
    AABB bounds;
  };


  struct Data {
    std::vector<Vector3> positions;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<Vector3> tangents;
    std::variant<std::vector<std::uint16_t>, std::vector<std::uint32_t>> indices;
    std::vector<MaterialSlotInfo> material_slots;
    std::vector<SubMeshInfo> sub_meshes;
  };


  struct SubmeshData {
    std::vector<Vector3> positions;
    std::vector<Vector3> normals;
    std::vector<Vector3> tangents;
    std::vector<Vector2> uvs;
    std::vector<MeshletData> meshlets;
    std::vector<std::uint8_t> vertex_indices;
    std::vector<MeshletTriangleIndexData> triangle_indices;
    std::uint32_t material_idx;
    bool idx32;
  };


  struct Data2 {
    std::vector<MaterialSlotInfo> material_slots;
    std::vector<SubmeshData> submeshes;
  };


  static constexpr std::size_t meshlet_max_verts_{128};
  static constexpr std::size_t meshlet_max_prims_{256};

private:
  std::unique_ptr<GeometryData> m_cpu_data_{nullptr};
  std::vector<SubMeshInfo> m_submeshes_;
  std::vector<MaterialSlotInfo> m_mtl_slots_;
  AABB m_bounds_{};
  graphics::SharedDeviceChildHandle<graphics::Buffer> pos_buf_;
  graphics::SharedDeviceChildHandle<graphics::Buffer> norm_buf_;
  graphics::SharedDeviceChildHandle<graphics::Buffer> tan_buf_;
  graphics::SharedDeviceChildHandle<graphics::Buffer> uv_buf_;
  graphics::SharedDeviceChildHandle<graphics::Buffer> idx_buf_;
  int m_vertex_count_{0};
  int m_index_count_{0};
  int m_submesh_count_{0};
  DXGI_FORMAT m_idx_format_{DXGI_FORMAT_R16_UINT};

  auto UploadToGpu() noexcept -> void;
  auto CalculateBounds() noexcept -> void;
  auto EnsureCpuMemory() noexcept -> void;
  auto Set16BitIndicesFrom32BitBuffer(std::span<std::uint32_t const> indices) noexcept -> void;

public:
  LEOPPHAPI auto OnDrawProperties(bool& changed) -> void override;

  Mesh() = default;
  LEOPPHAPI explicit Mesh(Data data, bool keep_data_in_cpu_memory = false) noexcept;
  Mesh(Mesh const&) = delete;
  Mesh(Mesh&& other) noexcept = delete;

  LEOPPHAPI ~Mesh() override;

  auto operator=(Mesh const&) -> void = delete;
  auto operator=(Mesh&& other) noexcept -> void = delete;

  [[nodiscard]] LEOPPHAPI auto GetPositions() const noexcept -> std::span<Vector3 const>;
  LEOPPHAPI auto SetPositions(std::span<Vector3 const> positions) noexcept -> void;
  LEOPPHAPI auto SetPositions(std::vector<Vector3>&& positions) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetNormals() const noexcept -> std::span<Vector3 const>;
  LEOPPHAPI auto SetNormals(std::span<Vector3 const> normals) noexcept -> void;
  LEOPPHAPI auto SetNormals(std::vector<Vector3>&& normals) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetUVs() const noexcept -> std::span<Vector2 const>;
  LEOPPHAPI auto SetUVs(std::span<Vector2 const> uvs) noexcept -> void;
  LEOPPHAPI auto SetUVs(std::vector<Vector2>&& uvs) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetTangents() const noexcept -> std::span<Vector3 const>;
  LEOPPHAPI auto SetTangents(std::span<Vector3 const> tangents) noexcept -> void;
  LEOPPHAPI auto SetTangents(std::vector<Vector3>&& tangents) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetIndices16() const noexcept -> std::span<std::uint16_t const>;
  [[nodiscard]] LEOPPHAPI auto GetIndices32() const noexcept -> std::span<std::uint32_t const>;
  LEOPPHAPI auto SetIndices(std::span<std::uint16_t const> indices) noexcept -> void;
  LEOPPHAPI auto SetIndices(std::span<std::uint32_t const> indices) noexcept -> void;
  LEOPPHAPI auto SetIndices(std::vector<std::uint16_t>&& indices) noexcept -> void;
  LEOPPHAPI auto SetIndices(std::vector<std::uint32_t>&& indices) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetMaterialSlots() const noexcept -> std::span<MaterialSlotInfo const>;
  LEOPPHAPI auto SetMaterialSlots(std::span<MaterialSlotInfo const> mtl_slots) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetSubMeshes() const noexcept -> std::span<SubMeshInfo const>;
  LEOPPHAPI auto SetSubMeshes(std::span<SubMeshInfo const> submeshes) noexcept -> void;
  LEOPPHAPI auto SetSubmeshes(std::vector<SubMeshInfo>&& submeshes) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetBounds() const noexcept -> AABB const&;

  LEOPPHAPI auto SetData(Data const& data) noexcept -> void;
  LEOPPHAPI auto SetData(Data&& data) noexcept -> void;
  [[nodiscard]] LEOPPHAPI auto ValidateAndUpdate(bool keep_data_in_cpu_memory = false) noexcept -> bool;

  [[nodiscard]] LEOPPHAPI auto HasCpuMemory() const noexcept -> bool;
  LEOPPHAPI auto ReleaseCpuMemory() noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetPositionBuffer() const noexcept -> graphics::SharedDeviceChildHandle<graphics::Buffer>
    const&;
  [[nodiscard]] LEOPPHAPI auto GetNormalBuffer() const noexcept -> graphics::SharedDeviceChildHandle<graphics::Buffer>
    const&;
  [[nodiscard]] LEOPPHAPI auto GetUvBuffer() const noexcept -> graphics::SharedDeviceChildHandle<graphics::Buffer> const
    &;
  [[nodiscard]] LEOPPHAPI auto GetTangentBuffer() const noexcept -> graphics::SharedDeviceChildHandle<graphics::Buffer>
    const&;
  [[nodiscard]] LEOPPHAPI auto GetIndexBuffer() const noexcept -> graphics::SharedDeviceChildHandle<graphics::Buffer>
    const&;

  [[nodiscard]] LEOPPHAPI auto GetVertexCount() const noexcept -> int;
  [[nodiscard]] LEOPPHAPI auto GetIndexCount() const noexcept -> int;
  [[nodiscard]] LEOPPHAPI auto GetSubmeshCount() const noexcept -> int;

  [[nodiscard]] LEOPPHAPI auto GetIndexFormat() const noexcept -> DXGI_FORMAT;
};
}

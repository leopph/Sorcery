#include "ResourceManager.hpp"
#include "app.hpp"
#include "ExternalResource.hpp"
#include "FileIo.hpp"
#include "job_system.hpp"
#include "MemoryAllocation.hpp"
#include "Reflection.hpp"
#include "rendering/render_manager.hpp"
#include "Resources/Scene.hpp"

#include <DirectXTex.h>
#include <wrl/client.h>

#include <bit>
#include <cassert>
#include <iostream>
#include <ranges>
#include <utility>

using Microsoft::WRL::ComPtr;


namespace sorcery {
namespace {
namespace {
std::vector const kQuadPositions{Vector3{-1, 1, 0}, Vector3{-1, -1, 0}, Vector3{1, -1, 0}, Vector3{1, 1, 0}};
std::vector const kQuadUvs{Vector2{0, 0}, Vector2{0, 1}, Vector2{1, 1}, Vector2{1, 0}};
std::vector<std::uint32_t> const kQuadIndices{2, 1, 0, 0, 3, 2};
std::vector const kCubePositions{
  Vector3{0.5f, 0.5f, 0.5f}, Vector3{0.5f, 0.5f, 0.5f}, Vector3{0.5f, 0.5f, 0.5f}, Vector3{-0.5f, 0.5f, 0.5f},
  Vector3{-0.5f, 0.5f, 0.5f}, Vector3{-0.5f, 0.5f, 0.5f}, Vector3{-0.5f, 0.5f, -0.5f}, Vector3{-0.5f, 0.5f, -0.5f},
  Vector3{-0.5f, 0.5f, -0.5f}, Vector3{0.5f, 0.5f, -0.5f}, Vector3{0.5f, 0.5f, -0.5f}, Vector3{0.5f, 0.5f, -0.5f},
  Vector3{0.5f, -0.5f, 0.5f}, Vector3{0.5f, -0.5f, 0.5f}, Vector3{0.5f, -0.5f, 0.5f}, Vector3{-0.5f, -0.5f, 0.5f},
  Vector3{-0.5f, -0.5f, 0.5f}, Vector3{-0.5f, -0.5f, 0.5f}, Vector3{-0.5f, -0.5f, -0.5f}, Vector3{-0.5f, -0.5f, -0.5f},
  Vector3{-0.5f, -0.5f, -0.5f}, Vector3{0.5f, -0.5f, -0.5f}, Vector3{0.5f, -0.5f, -0.5f}, Vector3{0.5f, -0.5f, -0.5f},
};
std::vector const kCubeUvs{
  Vector2{1, 0}, Vector2{1, 0}, Vector2{0, 0}, Vector2{0, 0}, Vector2{0, 0}, Vector2{1, 0}, Vector2{1, 0},
  Vector2{0, 1}, Vector2{0, 0}, Vector2{0, 0}, Vector2{1, 1}, Vector2{1, 0}, Vector2{1, 1}, Vector2{1, 1},
  Vector2{0, 1}, Vector2{0, 1}, Vector2{0, 1}, Vector2{1, 1}, Vector2{1, 1}, Vector2{0, 0}, Vector2{0, 1},
  Vector2{0, 1}, Vector2{1, 0}, Vector2{1, 1}
};
std::vector<std::uint32_t> const kCubeIndices{
  // Top face
  7, 4, 1, 1, 10, 7,
  // Bottom face
  16, 19, 22, 22, 13, 16,
  // Front face
  23, 20, 8, 8, 11, 23,
  // Back face
  17, 14, 2, 2, 5, 17,
  // Right face
  21, 9, 0, 0, 12, 21,
  // Left face
  15, 3, 6, 6, 18, 15
};
}
}


auto ResourceManager::ResourceGuidLess::operator()(std::unique_ptr<Resource> const& lhs,
                                                   std::unique_ptr<Resource> const& rhs) const noexcept -> bool {
  return lhs->GetGuid() < rhs->GetGuid();
}


auto ResourceManager::ResourceGuidLess::operator()(std::unique_ptr<Resource> const& lhs,
                                                   Guid const& rhs) const noexcept -> bool {
  return lhs->GetGuid() < rhs;
}


auto ResourceManager::ResourceGuidLess::operator()(Guid const& lhs,
                                                   std::unique_ptr<Resource> const& rhs) const noexcept -> bool {
  return lhs < rhs->GetGuid();
}


auto ResourceManager::InternalLoadResource(Guid const& guid, ResourceDescription const& desc) -> ObserverPtr<Resource> {
  ObserverPtr<Job> loader_job;

  std::cout << "loading " << desc.name << '\n';

  {
    auto loader_jobs{loader_jobs_.Lock()};

    if (auto const it{loader_jobs->find(guid)}; it != loader_jobs->end()) {
      loader_job = it->second;
    } else {
      loader_job = job_system_->CreateJob([this, &guid, &desc] {
        std::unique_ptr<Resource> res;

        if (desc.pathAbs.extension() == EXTERNAL_RESOURCE_EXT) {
          std::vector<std::uint8_t> fileBytes;

          if (!ReadFileBinary(desc.pathAbs, fileBytes)) {
            return;
          }

          ExternalResourceCategory resCat;
          std::vector<std::byte> resBytes;

          if (!UnpackExternalResource(as_bytes(std::span{fileBytes}), resCat, resBytes)) {
            return;
          }

          switch (resCat) {
            case ExternalResourceCategory::Texture: {
              res = LoadTexture(resBytes);
              break;
            }

            case ExternalResourceCategory::Mesh: {
              res = LoadMesh(resBytes);
              break;
            }
          }
        } else if (desc.pathAbs.extension() == SCENE_RESOURCE_EXT) {
          res = CreateDeserialize<Scene>(YAML::LoadFile(desc.pathAbs.string()));
        } else if (desc.pathAbs.extension() == MATERIAL_RESOURCE_EXT) {
          res = CreateDeserialize<Material>(YAML::LoadFile(desc.pathAbs.string()));
        }

        if (res) {
          res->SetGuid(guid);
          res->SetName(desc.name);

          auto const [it, inserted]{loaded_resources_.Lock()->emplace(std::move(res))};
          assert(inserted);
        }
      });
      job_system_->Run(loader_job);
      loader_jobs->emplace(guid, loader_job);
    }
  }

  assert(loader_job);
  job_system_->Wait(loader_job);

  {
    loader_jobs_.Lock()->erase(guid);
  }

  {
    auto const resources{loaded_resources_.LockShared()};
    auto const it{resources->find(guid)};
    return ObserverPtr{it != resources->end() ? it->get() : nullptr};
  }
}


auto ResourceManager::LoadTexture(
  std::span<std::byte const> const bytes) noexcept -> MaybeNull<std::unique_ptr<Resource>> {
  DirectX::TexMetadata meta;
  DirectX::ScratchImage img;
  if (FAILED(LoadFromDDSMemory(bytes.data(), bytes.size(), DirectX::DDS_FLAGS_NONE, &meta, img))) {
    return nullptr;
  }

  auto tex{App::Instance().GetRenderManager().CreateReadOnlyTexture(img)};

  if (!tex) {
    return nullptr;
  }

  std::unique_ptr<Resource> ret;

  if (meta.dimension == DirectX::TEX_DIMENSION_TEXTURE2D) {
    if (meta.IsCubemap()) {
      ret = Create<Cubemap>(std::move(tex));
    } else {
      ret = Create<Texture2D>(std::move(tex));
    }
  } else {
    return nullptr;
  }

  return ret;
}


auto ResourceManager::LoadMesh(std::span<std::byte const> const bytes) -> MaybeNull<std::unique_ptr<Resource>> {
  // Read meshletized data

  auto cur_bytes{as_bytes(std::span{bytes})};
  std::uint64_t mtl_count;

  if (!DeserializeFromBinary(cur_bytes, mtl_count)) {
    return nullptr;
  }

  cur_bytes = cur_bytes.subspan(sizeof mtl_count);
  std::uint64_t submesh_count;

  if (!DeserializeFromBinary(cur_bytes, submesh_count)) {
    return nullptr;
  }

  cur_bytes = cur_bytes.subspan(sizeof submesh_count);

  Mesh::Data2 mesh_data2;
  mesh_data2.material_slots.resize(mtl_count);

  for (auto i{0ull}; i < mtl_count; i++) {
    if (!DeserializeFromBinary(cur_bytes, mesh_data2.material_slots[i].name)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(mesh_data2.material_slots[i].name.size() + 8);
  }

  mesh_data2.submeshes.resize(submesh_count);

  for (auto i{0ull}; i < submesh_count; i++) {
    std::uint64_t vertex_count;

    if (!DeserializeFromBinary(cur_bytes, vertex_count)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof vertex_count);
    std::uint64_t meshlet_count;

    if (!DeserializeFromBinary(cur_bytes, meshlet_count)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof meshlet_count);
    std::uint64_t vtx_ib_byte_count;

    if (!DeserializeFromBinary(cur_bytes, vtx_ib_byte_count)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof vtx_ib_byte_count);
    std::uint64_t tri_index_count;

    if (!DeserializeFromBinary(cur_bytes, tri_index_count)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof tri_index_count);
    std::uint32_t mtl_idx;

    if (!DeserializeFromBinary(cur_bytes, mtl_idx)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof mtl_idx);
    bool idx32;

    if (!DeserializeFromBinary(cur_bytes, idx32)) {
      return nullptr;
    }

    cur_bytes = cur_bytes.subspan(sizeof idx32);

    mesh_data2.submeshes[i].positions.resize(vertex_count);
    std::memcpy(mesh_data2.submeshes[i].positions.data(), cur_bytes.data(), vertex_count * sizeof(Vector3));
    cur_bytes = cur_bytes.subspan(vertex_count * sizeof(Vector3));

    mesh_data2.submeshes[i].normals.resize(vertex_count);
    std::memcpy(mesh_data2.submeshes[i].normals.data(), cur_bytes.data(), vertex_count * sizeof(Vector3));
    cur_bytes = cur_bytes.subspan(vertex_count * sizeof(Vector3));

    mesh_data2.submeshes[i].tangents.resize(vertex_count);
    std::memcpy(mesh_data2.submeshes[i].tangents.data(), cur_bytes.data(), vertex_count * sizeof(Vector3));
    cur_bytes = cur_bytes.subspan(vertex_count * sizeof(Vector3));

    mesh_data2.submeshes[i].uvs.resize(vertex_count);
    std::memcpy(mesh_data2.submeshes[i].uvs.data(), cur_bytes.data(), vertex_count * sizeof(Vector2));
    cur_bytes = cur_bytes.subspan(vertex_count * sizeof(Vector2));

    mesh_data2.submeshes[i].meshlets.resize(meshlet_count);
    std::memcpy(mesh_data2.submeshes[i].meshlets.data(), cur_bytes.data(), meshlet_count * sizeof(Mesh::MeshletData));
    cur_bytes = cur_bytes.subspan(meshlet_count * sizeof(Mesh::MeshletData));

    mesh_data2.submeshes[i].vertex_indices.resize(vtx_ib_byte_count);
    std::memcpy(mesh_data2.submeshes[i].vertex_indices.data(), cur_bytes.data(), vtx_ib_byte_count);
    cur_bytes = cur_bytes.subspan(vtx_ib_byte_count);

    mesh_data2.submeshes[i].triangle_indices.resize(tri_index_count);
    std::memcpy(mesh_data2.submeshes[i].triangle_indices.data(), cur_bytes.data(),
      tri_index_count * sizeof(Mesh::MeshletTriangleIndexData));
    cur_bytes = cur_bytes.subspan(tri_index_count * sizeof(Mesh::MeshletTriangleIndexData));

    mesh_data2.submeshes[i].material_idx = mtl_idx;
    mesh_data2.submeshes[i].idx32 = idx32;
  }

  // Flatten meshletized data TODO do this only when mesh shaders aren't supported

  Mesh::Data mesh_data;
  mesh_data.material_slots = std::move(mesh_data2.material_slots);
  mesh_data.indices.emplace<std::vector<std::uint32_t>>();

  for (auto& [positions, normals, tangents, uvs, meshlets, vertex_indices, triangle_indices, material_idx, idx32] :
       mesh_data2.submeshes) {
    // Flatten indices

    std::vector<std::uint32_t> indices;

    std::variant<std::uint16_t const*, std::uint32_t const*> vtx_index_list_ptr_variant;
    if (idx32) {
      // 32 bit indices
      vtx_index_list_ptr_variant.emplace<std::uint32_t const*>(
        std::bit_cast<std::uint32_t const*>(vertex_indices.data()));
    } else {
      // 16 bit indices
      vtx_index_list_ptr_variant.emplace<std::uint16_t const*>(
        std::bit_cast<std::uint16_t const*>(vertex_indices.data()));
    }

    std::visit([&meshlets, &triangle_indices, &indices](auto const vtx_index_list_ptr) {
      for (auto const& [vert_count, vert_offset, prim_count, prim_offset] : meshlets) {
        for (std::size_t i{0}; i < prim_count; i++) {
          auto const& [idx0, idx1, idx2]{triangle_indices[prim_offset + i]};

          indices.emplace_back(static_cast<std::uint32_t>(vtx_index_list_ptr[vert_offset + idx0]));
          indices.emplace_back(static_cast<std::uint32_t>(vtx_index_list_ptr[vert_offset + idx1]));
          indices.emplace_back(static_cast<std::uint32_t>(vtx_index_list_ptr[vert_offset + idx2]));
        }
      }
    }, vtx_index_list_ptr_variant);

    // Store data in DrawIndexedInstanced-compatible format

    mesh_data.sub_meshes.emplace_back(static_cast<int>(mesh_data.positions.size()),
      std::visit([]<typename T>(std::vector<T> const& index_list) { return static_cast<int>(index_list.size()); },
        mesh_data.indices), static_cast<int>(indices.size()), static_cast<int>(material_idx), AABB{});

    std::ranges::copy(positions, std::back_inserter(mesh_data.positions));
    std::ranges::copy(normals, std::back_inserter(mesh_data.normals));
    std::ranges::copy(uvs, std::back_inserter(mesh_data.uvs));
    std::ranges::copy(tangents, std::back_inserter(mesh_data.tangents));
    std::ranges::copy(indices, std::back_inserter(std::get<std::vector<std::uint32_t>>(mesh_data.indices)));
  }

  // Transform indices to 16-bit if possible

  if (auto const& indices32{std::get<std::vector<std::uint32_t>>(mesh_data.indices)}; std::ranges::all_of(indices32,
    [](std::uint32_t const idx) {
      return idx <= std::numeric_limits<std::uint16_t>::max();
    })) {
    std::vector<std::uint16_t> indices16;
    indices16.reserve(indices32.size());

    std::ranges::transform(indices32, std::back_inserter(indices16), [](std::uint32_t const idx) {
      return static_cast<std::uint16_t>(idx);
    });

    mesh_data.indices = std::move(indices16);
  }

  return Create<Mesh>(std::move(mesh_data));
}


ResourceManager::ResourceManager(JobSystem& job_system) :
  job_system_{&job_system} {}


auto ResourceManager::Unload(Guid const& guid) -> void {
  auto resources{loaded_resources_.Lock()};

  if (auto const it{resources->find(guid)}; it != std::end(*resources)) {
    resources->erase(it);
  }
}


auto ResourceManager::UnloadAll() -> void {
  loaded_resources_.Lock()->clear();
}


auto ResourceManager::IsLoaded(Guid const& guid) -> bool {
  for (auto const& res : default_resources_) {
    if (res->GetGuid() == guid) {
      return true;
    }
  }

  return loaded_resources_.LockShared()->contains(guid);
}


auto ResourceManager::UpdateMappings(std::map<Guid, ResourceDescription> mappings) -> void {
  while (true) {
    if (auto self_mappings{mappings_.TryLock()}) {
      **self_mappings = std::move(mappings);
      break;
    }
  }
}


auto ResourceManager::GetGuidsForResourcesOfType(rttr::type const& type,
                                                 std::vector<Guid>& out) noexcept -> void {
  // Default resources
  for (auto const& res : default_resources_) {
    if (rttr::type::get(*res).is_derived_from(type)) {
      out.emplace_back(res->GetGuid());
    }
  }

  // File mappings
  for (auto const& [guid, desc] : *mappings_.LockShared()) {
    if (desc.type.is_derived_from(type)) {
      out.emplace_back(guid);
    }
  }

  // Other, loaded resources that don't come from files
  for (auto const& res : *loaded_resources_.LockShared()) {
    auto contains{false};
    for (auto const& guid : out) {
      if (guid == res->GetGuid()) {
        contains = true;
        break;
      }
    }

    if (!contains && rttr::type::get(*res).is_derived_from(type)) {
      out.emplace_back(res->GetGuid());
    }
  }
}


auto ResourceManager::GetInfoForResourcesOfType(rttr::type const& type, std::vector<ResourceInfo>& out) -> void {
  // Default resources
  for (auto const& res : default_resources_) {
    if (auto const res_type{rttr::type::get(*res)}; res_type.is_derived_from(type)) {
      out.emplace_back(res->GetGuid(), res->GetName(), res_type);
    }
  }

  // File mappings
  for (auto const& [guid, desc] : *mappings_.LockShared()) {
    if (desc.type.is_derived_from(type)) {
      out.emplace_back(guid, desc.name, desc.type);
    }
  }

  // Other, loaded resources that don't come from files
  for (auto const& res : *loaded_resources_.LockShared()) {
    auto contains{false};
    for (auto const& res_info : out) {
      if (res_info.guid == res->GetGuid()) {
        contains = true;
        break;
      }
    }

    if (!contains && rttr::type::get(*res).is_derived_from(type)) {
      out.emplace_back(res->GetGuid(), res->GetName(), res->get_type());
    }
  }
}


auto ResourceManager::GetDefaultMaterial() const noexcept -> ObserverPtr<Material> {
  return ObserverPtr{default_mtl_.get()};
}


auto ResourceManager::GetCubeMesh() const noexcept -> ObserverPtr<Mesh> {
  return ObserverPtr{cube_mesh_.get()};
}


auto ResourceManager::GetPlaneMesh() const noexcept -> ObserverPtr<Mesh> {
  return ObserverPtr{plane_mesh_.get()};
}


auto ResourceManager::GetSphereMesh() const noexcept -> ObserverPtr<Mesh> {
  return ObserverPtr{sphere_mesh_.get()};
}


auto ResourceManager::CreateDefaultResources() -> void {
  if (!default_mtl_) {
    default_mtl_ = Create<Material>();
    default_mtl_->SetGuid(default_mtl_guid_);
    default_mtl_->SetName("Default Material");
    default_resources_.emplace_back(default_mtl_.get());
  }

  if (!cube_mesh_) {
    std::vector<Vector3> cubeNormals;
    CalculateNormals(kCubePositions, kCubeIndices, cubeNormals);

    std::vector<Vector3> cubeTangents;
    CalculateTangents(kCubePositions, kCubeUvs, kCubeIndices, cubeTangents);

    cube_mesh_ = Create<Mesh>();
    cube_mesh_->SetGuid(cube_mesh_guid_);
    cube_mesh_->SetName("Cube");
    cube_mesh_->SetPositions(kCubePositions);
    cube_mesh_->SetNormals(std::move(cubeNormals));
    cube_mesh_->SetUVs(kCubeUvs);
    cube_mesh_->SetTangents(std::move(cubeTangents));
    cube_mesh_->SetIndices(kCubeIndices);
    cube_mesh_->SetMaterialSlots(std::array{Mesh::MaterialSlotInfo{"Material"}});
    cube_mesh_->SetSubMeshes(std::array{Mesh::SubMeshInfo{0, 0, static_cast<int>(kCubeIndices.size()), 0, AABB{}}});
    if (!cube_mesh_->ValidateAndUpdate(false)) {
      throw std::runtime_error{"Failed to validate and update default cube mesh."};
    }
    default_resources_.emplace_back(cube_mesh_.get());
  }

  if (!plane_mesh_) {
    std::vector<Vector3> quadNormals;
    CalculateNormals(kQuadPositions, kQuadIndices, quadNormals);

    std::vector<Vector3> quadTangents;
    CalculateTangents(kQuadPositions, kQuadUvs, kQuadIndices, quadTangents);

    plane_mesh_ = Create<Mesh>();
    plane_mesh_->SetGuid(plane_mesh_guid_);
    plane_mesh_->SetName("Plane");
    plane_mesh_->SetPositions(kQuadPositions);
    plane_mesh_->SetNormals(std::move(quadNormals));
    plane_mesh_->SetUVs(kQuadUvs);
    plane_mesh_->SetTangents(std::move(quadTangents));
    plane_mesh_->SetIndices(kQuadIndices);
    plane_mesh_->SetMaterialSlots(std::array{Mesh::MaterialSlotInfo{"Material"}});
    plane_mesh_->SetSubMeshes(std::array{Mesh::SubMeshInfo{0, 0, static_cast<int>(kQuadIndices.size()), 0, AABB{}}});
    if (!plane_mesh_->ValidateAndUpdate(false)) {
      throw std::runtime_error{"Failed to validate and update default plane mesh."};
    }
    default_resources_.emplace_back(plane_mesh_.get());
  }

  if (!sphere_mesh_) {
    sphere_mesh_ = Create<Mesh>();
    sphere_mesh_->SetGuid(sphere_mesh_guid_);
    sphere_mesh_->SetName("Sphere");
    std::vector<Vector3> spherePositions;
    std::vector<Vector3> sphereNormals;
    std::vector<Vector3> sphereTangents;
    std::vector<Vector2> sphereUvs;
    std::vector<std::uint32_t> sphereIndices;
    rendering::GenerateSphereMesh(1, 50, 50, spherePositions, sphereNormals, sphereUvs, sphereIndices);
    auto const sphereIdxCount{std::size(sphereIndices)};
    CalculateTangents(spherePositions, sphereUvs, sphereIndices, sphereTangents);
    sphere_mesh_->SetPositions(std::move(spherePositions));
    sphere_mesh_->SetNormals(std::move(sphereNormals));
    sphere_mesh_->SetUVs(std::move(sphereUvs));
    sphere_mesh_->SetTangents(std::move(sphereTangents));
    sphere_mesh_->SetIndices(std::move(sphereIndices));
    sphere_mesh_->SetMaterialSlots(std::array{Mesh::MaterialSlotInfo{"Material"}});
    sphere_mesh_->SetSubMeshes(std::array{Mesh::SubMeshInfo{0, 0, static_cast<int>(sphereIdxCount), 0, AABB{}}});
    if (!sphere_mesh_->ValidateAndUpdate(false)) {
      throw std::runtime_error{"Failed to validate and update default sphere mesh."};
    }
    default_resources_.emplace_back(sphere_mesh_.get());
  }
}
}

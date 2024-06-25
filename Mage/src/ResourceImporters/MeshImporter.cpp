#include "MeshImporter.hpp"
#include "Serialization.hpp"
#include "../FileIo.hpp"
#include "../Resources/Mesh.hpp"

#include <DirectXMesh.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <ranges>

RTTR_REGISTRATION {
  rttr::registration::class_<sorcery::MeshImporter>{"Mesh Importer"}
    .REFLECT_REGISTER_RESOURCE_IMPORTER_CTOR;
}


namespace sorcery {
namespace {
[[nodiscard]] auto Convert(aiVector3D const& aiVec) noexcept -> Vector3 {
  return Vector3{aiVec.x, aiVec.y, aiVec.z};
}


[[nodiscard]] auto Convert(aiMatrix4x4 const& aiMat) noexcept -> Matrix4 {
  return Matrix4{
    aiMat.a1, aiMat.a2, aiMat.a3, aiMat.a4,
    aiMat.b1, aiMat.b2, aiMat.b3, aiMat.b4,
    aiMat.c1, aiMat.c2, aiMat.c3, aiMat.c4,
    aiMat.d1, aiMat.d2, aiMat.d3, aiMat.d4
  };
}
}


auto MeshImporter::GetSupportedFileExtensions(std::pmr::vector<std::string>& out) -> void {
  std::string extensions;
  Assimp::Importer const importer;
  importer.GetExtensionList(extensions);

  // Assimp extension list format is "*.3ds;*.obj;*.dae"
  for (auto const ext : std::views::split(extensions, std::string_view{";"})) {
    out.emplace_back(std::string_view{std::begin(ext), std::end(ext)}.substr(1));
  }
}


auto MeshImporter::Import(std::filesystem::path const& src, std::vector<std::byte>& bytes,
                          ExternalResourceCategory& categ) -> bool {
  std::vector<unsigned char> meshBytes;

  if (!ReadFileBinary(src, meshBytes)) {
    return false;
  }

  Assimp::Importer importer;

  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
    aiComponent_ANIMATIONS | aiComponent_BONEWEIGHTS | aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_COLORS);
  importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
  importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80.0f);

  auto const scene{
    importer.ReadFileFromMemory(meshBytes.data(), meshBytes.size(),
      aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_ConvertToLeftHanded | aiProcess_TransformUVCoords |
      aiProcess_RemoveComponent)
  };

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    throw std::runtime_error{std::format("Failed to import model at {}: {}.", src.string(), importer.GetErrorString())};
  }

  Mesh::Data2 mesh_data;

  // Collect all info from aiMaterials

  mesh_data.material_slots.resize(scene->mNumMaterials);

  for (unsigned i{0}; i < scene->mNumMaterials; i++) {
    mesh_data.material_slots[i].name = scene->mMaterials[i]->GetName().C_Str();
  }

  // Collect all info from aiMeshes

  struct MeshProcessingData {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<Vector3> tangents;
    std::vector<std::uint32_t> indices;
    unsigned mtl_idx{};
  };

  // Collected mesh data not yet transformed by node matrices and only containing 32 bit indices
  std::vector<MeshProcessingData> meshes_untransformed;
  meshes_untransformed.reserve(scene->mNumMeshes);

  for (unsigned i = 0; i < scene->mNumMeshes; i++) {
    // These meshes are always triangle-only, because
    // AI_CONFIG_PP_SBP_REMOVE is set to remove points and lines, 
    // aiProcess_Triangulate splits up primitives with more than 3 vertices
    // aiProcess_SortByPType splits up meshes with more than 1 primitive type into homogeneous ones
    // TODO Implement non-triangle rendering support

    aiMesh const* const mesh{scene->mMeshes[i]};
    auto& [vertices, normals, uvs, tangents, indices, mtlIdx]{meshes_untransformed.emplace_back()};

    if (!mesh->HasPositions() || !mesh->HasNormals() || !mesh->HasTextureCoords(0) || !mesh->
        HasTangentsAndBitangents()) {
      // TODO log or something
      continue;
    }

    vertices.reserve(mesh->mNumVertices);
    normals.reserve(mesh->mNumVertices);
    uvs.reserve(mesh->mNumVertices);
    tangents.reserve(mesh->mNumVertices);

    for (unsigned j = 0; j < mesh->mNumVertices; j++) {
      vertices.emplace_back(Convert(mesh->mVertices[j]));
      normals.emplace_back(Normalized(Convert(mesh->mNormals[j])));
      uvs.emplace_back(mesh->HasTextureCoords(0) ? Vector2{Convert(mesh->mTextureCoords[0][j])} : Vector2{});
      tangents.emplace_back(Convert(mesh->mTangents[j]));
    }

    for (unsigned j = 0; j < mesh->mNumFaces; j++) {
      std::ranges::copy(std::span{mesh->mFaces[j].mIndices, mesh->mFaces[j].mNumIndices}, std::back_inserter(indices));
    }

    mtlIdx = mesh->mMaterialIndex;
  }

  // Accumulate node trafos

  struct MeshTrafoAndIndex {
    Matrix4 trafo;
    unsigned mesh_idx;
  };

  struct NodeAndAccumTrafo {
    Matrix4 accum_parent_trafo;
    aiNode const* node;
  };

  std::vector<MeshTrafoAndIndex> mesh_indices_with_trafos;
  std::queue<NodeAndAccumTrafo> transform_queue;
  transform_queue.emplace(Matrix4::Identity(), scene->mRootNode);

  while (!transform_queue.empty()) {
    auto const& [accumParentTrafo, node] = transform_queue.front();
    auto const trafo{Convert(node->mTransformation).Transpose() * accumParentTrafo};

    for (unsigned i = 0; i < node->mNumMeshes; ++i) {
      mesh_indices_with_trafos.emplace_back(trafo, node->mMeshes[i]);
    }

    for (unsigned i = 0; i < node->mNumChildren; ++i) {
      transform_queue.emplace(trafo, node->mChildren[i]);
    }

    transform_queue.pop();
  }

  // Transform mesh geometry using trafos

  // Collected mesh data transformed by node matrices
  std::vector<MeshProcessingData> meshes_transformed;

  for (auto const& [trafo, meshIdx] : mesh_indices_with_trafos) {
    auto& [vertices, normals, uvs, tangents, indices, mtlIdx]{
      meshes_transformed.emplace_back(meshes_untransformed[meshIdx])
    };

    Matrix4 const trafo_inv_transp{trafo.Inverse().Transpose()};

    for (int i = 0; i < std::ssize(vertices); i++) {
      vertices[i] = Vector3{Vector4{vertices[i], 1} * trafo};
      normals[i] = Vector3{Vector4{normals[i], 0} * trafo_inv_transp};
      tangents[i] = Vector3{Vector4{tangents[i], 0} * trafo_inv_transp};
    }
  }

  // Generate meshlets and final mesh data

  for (auto& [vertices, normals, uvs, tangents, indices, mtlIdx] : meshes_transformed) {
    // Convert indices to 16 bit if possible to save space

    std::variant<std::vector<std::uint16_t>, std::vector<std::uint32_t>> indices_variant;

    if (std::ranges::all_of(indices, [](std::uint32_t const idx) {
      return idx <= std::numeric_limits<std::uint16_t>::max();
    })) {
      auto& indices16{indices_variant.emplace<std::vector<std::uint16_t>>()};
      std::ranges::transform(indices, std::back_inserter(indices16), [](std::uint32_t const idx) {
        return static_cast<std::uint16_t>(idx);
      });
    } else {
      indices_variant.emplace<std::vector<std::uint32_t>>(std::move(indices));
    }

    // Use the final index buffer to generate meshlets

    std::visit(
      [&vertices, &normals, &uvs, &tangents, mtlIdx, &mesh_data]<typename IndexList>(IndexList const& index_list) {
        auto const xm_vertices{std::make_unique_for_overwrite<DirectX::XMFLOAT3[]>(vertices.size())};
        std::ranges::transform(vertices, xm_vertices.get(), [](Vector3 const& vtx) {
          return DirectX::XMFLOAT3{vtx[0], vtx[1], vtx[2]};
        });

        std::vector<DirectX::Meshlet> meshlets;
        std::vector<std::uint8_t> unique_vertex_ib;
        std::vector<DirectX::MeshletTriangle> primitive_indices;

        DirectX::ComputeMeshlets(index_list.data(), index_list.size() / 3, xm_vertices.get(), vertices.size(), nullptr,
          meshlets, unique_vertex_ib, primitive_indices, Mesh::meshlet_max_verts_, Mesh::meshlet_max_prims_);

        std::vector<Mesh::MeshletData> meshlet_data;
        std::ranges::transform(meshlets, std::back_inserter(meshlet_data), [](DirectX::Meshlet const& meshlet) {
          return Mesh::MeshletData{meshlet.VertCount, meshlet.VertOffset, meshlet.PrimCount, meshlet.PrimOffset};
        });

        std::vector<Mesh::MeshletTriangleIndexData> triangle_index_data;
        std::ranges::transform(primitive_indices, std::back_inserter(triangle_index_data),
          [](DirectX::MeshletTriangle const& tri) { return Mesh::MeshletTriangleIndexData{tri.i0, tri.i1, tri.i2}; });

        mesh_data.submeshes.emplace_back(std::move(vertices), std::move(normals), std::move(tangents), std::move(uvs),
          std::move(meshlet_data), std::move(unique_vertex_ib), std::move(triangle_index_data), mtlIdx,
          sizeof(IndexList::value_type) == 4);
      }, indices_variant);
  }

  // Serialize

  SerializeToBinary(mesh_data.material_slots.size(), bytes);
  SerializeToBinary(mesh_data.submeshes.size(), bytes);

  for (auto const& [name] : mesh_data.material_slots) {
    SerializeToBinary(name, bytes);
  }

  for (auto const& [vertices, normals, tangents, uvs, meshlets, vertexIndices, triangleIndices, mtlIdx, idx32] :
       mesh_data.submeshes) {
    SerializeToBinary(vertices.size(), bytes);
    SerializeToBinary(meshlets.size(), bytes);
    SerializeToBinary(vertexIndices.size(), bytes);
    SerializeToBinary(triangleIndices.size(), bytes);
    SerializeToBinary(mtlIdx, bytes);
    SerializeToBinary(idx32, bytes);

    auto const vtx_bytes{as_bytes(std::span{vertices})};
    auto const norm_bytes{as_bytes(std::span{normals})};
    auto const tan_bytes{as_bytes(std::span{tangents})};
    auto const uv_bytes{as_bytes(std::span{uvs})};
    auto const meshlets_bytes{as_bytes(std::span{meshlets})};
    auto const vtx_ib_bytes{as_bytes(std::span{vertexIndices})};
    auto const tri_bytes{as_bytes(std::span{triangleIndices})};

    bytes.reserve(
      bytes.size() + vtx_bytes.size() + norm_bytes.size() + tan_bytes.size() + uv_bytes.size() + meshlets_bytes.size() +
      vtx_ib_bytes.size() + tri_bytes.size());

    std::ranges::copy(vtx_bytes, std::back_inserter(bytes));
    std::ranges::copy(norm_bytes, std::back_inserter(bytes));
    std::ranges::copy(tan_bytes, std::back_inserter(bytes));
    std::ranges::copy(uv_bytes, std::back_inserter(bytes));
    std::ranges::copy(meshlets_bytes, std::back_inserter(bytes));
    std::ranges::copy(vtx_ib_bytes, std::back_inserter(bytes));
    std::ranges::copy(tri_bytes, std::back_inserter(bytes));
  }

  categ = ExternalResourceCategory::Mesh;
  return true;
}


auto MeshImporter::GetImportedType(std::filesystem::path const& resPathAbs) noexcept -> rttr::type {
  return rttr::type::get<Mesh>();
}
}

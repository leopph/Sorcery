#pragma once

#include "MeshDataGroup.hpp"
#include "../Texture.hpp"
#include "../../math/Matrix.hpp"

#include <assimp/scene.h>

#include <filesystem>
#include <memory>


namespace leopph::internal
{
	class FileMeshDataGroup final : public MeshDataGroup
	{
		public:
			explicit FileMeshDataGroup(std::filesystem::path path);

			[[nodiscard]]
			const std::filesystem::path& Path() const;


		private:
			std::filesystem::path m_Path;

			std::vector<MeshData> ProcessNodes(const aiScene* scene) const;
			MeshData ProcessMesh(const aiMesh* mesh, const aiScene* scene, const Matrix3& trafo) const;
			std::shared_ptr<Texture> LoadTexture(const aiMaterial* material, aiTextureType type) const;
	};
}

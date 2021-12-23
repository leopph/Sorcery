#pragma once

#include "RenderComponent.hpp"
#include "../../api/LeopphApi.hpp"

#include <filesystem>
#include <memory>


namespace leopph
{
	class Model final : public internal::RenderComponent
	{
		public:
			// Load a Model from a file on disk.
			LEOPPHAPI Model(leopph::Entity* entity, std::filesystem::path path);

			Model(const Model& other) = delete;
			auto operator=(const Model& other) -> Model& = delete;

			Model(Model&& other) = delete;
			auto operator=(Model&& other) -> Model& = delete;

			LEOPPHAPI ~Model() override = default;

			// File path of the loaded Model.
			[[nodiscard]]
			auto Path() const -> const std::filesystem::path&;

		private:
			std::filesystem::path m_Path;

			[[nodiscard]]
			auto GetMeshData(const std::filesystem::path& path) const -> std::shared_ptr<internal::MeshDataGroup>;
	};
}

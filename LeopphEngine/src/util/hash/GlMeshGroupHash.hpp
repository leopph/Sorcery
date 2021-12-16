#pragma once

#include "../../rendering/geometry/GlMeshGroup.hpp"
#include "../../rendering/geometry/MeshDataGroup.hpp"

#include <functional>


namespace leopph::impl
{
	class GlMeshGroupHash
	{
	public:
		using is_transparent = void;

		std::size_t operator()(const GlMeshGroup& model) const;
		std::size_t operator()(const MeshDataGroup& modelData) const;


	private:
		std::hash<std::string> m_Hash;
	};
}
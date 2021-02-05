#pragma once

#include "vertex.h"
#include "texture.h"
#include "shader.h"

#include <vector>
#include <unordered_map>

namespace leopph::implementation
{
	// CLASS TO REPRESENT A MINIMAL DRAWABLE OBJECT
	class Mesh
	{
	private:
		static std::unordered_map<unsigned, size_t> s_Instances;

		unsigned m_VAO;
		unsigned m_VBO;
		unsigned m_EBO;

		std::vector<Vertex> m_Vertices;
		std::vector<unsigned> m_Indices;
		std::vector<Texture> m_Textures;

	public:
		Mesh(std::vector<Vertex> vertices, std::vector<unsigned> indices, std::vector<Texture> textures);
		~Mesh();

		Mesh(const Mesh& other);
		Mesh(Mesh&& other) noexcept;

		Mesh& operator=(const Mesh& other);
		Mesh& operator=(Mesh&& other) noexcept;

		bool operator==(const Mesh& other) const;

		void Draw(const Shader& shader) const;
	};
}
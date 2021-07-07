#include "skyboximpl.h"

#include "skybox.h"

#include "logger.h"

#include <glad/glad.h>
#include <stb_image.h>

#include <cstddef>
#include <functional>
#include <stdexcept>

leopph::impl::SkyboxImpl::SkyboxImpl(const std::filesystem::path& left, const std::filesystem::path& right, const std::filesystem::path& top, const std::filesystem::path& bottom, const std::filesystem::path& back, const std::filesystem::path& front) :
	fileNames{ left.string() + right.string() + top.string() + bottom.string() + back.string() + front.string() }
{
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &m_TexID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_TexID);

	const std::filesystem::path* paths[]{ &right, &left, &top, &bottom, &front, &back };

	int width, height, nrChannels;
	unsigned char* data{ nullptr };

	for (std::size_t i{ 0 }; const auto& path : paths)
	{
		data = stbi_load(path->string().data(), &width, &height, &nrChannels, 0);

		Logger::Instance().Debug(std::to_string(nrChannels));

		if (data == nullptr)
		{
			auto msg{ "Skybox texture on path [" + path->string() + "] could not be loaded." };
			Logger::Instance().Error(msg);
			glDeleteTextures(1, &m_TexID);
			throw std::runtime_error{ msg };
		}

		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + static_cast<GLenum>(i), 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		stbi_image_free(data);

		i++;
	}

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);

	glBindVertexArray(m_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, s_CubeVertices.size() * sizeof(decltype(s_CubeVertices)::value_type), s_CubeVertices.data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0/*3 * sizeof(decltype(s_CubeVertices)::value_type)*/, nullptr);

	glBindVertexArray(0);
}

leopph::impl::SkyboxImpl::~SkyboxImpl()
{
	glDeleteTextures(1, &m_TexID);
	glDeleteBuffers(1, &m_VBO);
	glDeleteVertexArrays(1, &m_VAO);
}

unsigned leopph::impl::SkyboxImpl::ID() const
{
	return m_TexID;
}

void leopph::impl::SkyboxImpl::Draw(const Shader& shader) const
{
	glActiveTexture(GL_TEXTURE0);
	shader.SetUniform("skybox", 0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_TexID);

	glBindVertexArray(m_VAO);
	glDepthMask(GL_FALSE);
	glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(s_CubeVertices.size()));
	glDepthMask(GL_TRUE);

	glBindVertexArray(0);
}

std::size_t leopph::impl::SkyboxImplHash::operator()(const SkyboxImpl& skyboxImpl) const
{
	return std::hash<std::string>{}(skyboxImpl.fileNames);
}

std::size_t leopph::impl::SkyboxImplHash::operator()(const Skybox& skybox) const
{
	return std::hash<std::string>{}(skybox.FileNames());
}

std::size_t leopph::impl::SkyboxImplHash::operator()(const std::string& string) const
{
	return std::hash<std::string>{}(string);
}

bool leopph::impl::SkyboxImplEqual::operator()(const SkyboxImpl& left, const SkyboxImpl& right) const
{
	return left.fileNames == right.fileNames;
}

bool leopph::impl::SkyboxImplEqual::operator()(const Skybox& left, const SkyboxImpl& right) const
{
	return left == right;
}

bool leopph::impl::SkyboxImplEqual::operator()(const SkyboxImpl& left, const Skybox& right) const
{
	return left == right;
}

bool leopph::impl::SkyboxImplEqual::operator()(const std::string& left, const SkyboxImpl& right) const
{
	return left == right.fileNames;
}

bool leopph::impl::SkyboxImplEqual::operator()(const SkyboxImpl& left, const std::string& right) const
{
	return left.fileNames == right;
}

const std::vector<float> leopph::impl::SkyboxImpl::s_CubeVertices
{
	-1.0f,  1.0f, -1.0f,
	-1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	-1.0f,  1.0f, -1.0f,
	 1.0f,  1.0f, -1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	 1.0f, -1.0f,  1.0f
};
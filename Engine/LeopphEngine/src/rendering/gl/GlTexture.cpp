#include "GlTexture.hpp"

#include "Logger.hpp"
#include "rendering/gl/GlCore.hpp"

#include <string>


namespace leopph
{
	GlTexture::GlTexture(Image const& img) :
		m_Texture{},
		m_Width{img.Width()},
		m_Height{img.Height()}
	{
		GLenum colorFormat, internalFormat;

		switch (img.Channels())
		{
			case 1:
			{
				colorFormat = GL_RED;
				internalFormat = GL_R8;
				break;
			}

			case 3:
			{
				colorFormat = GL_RGB;

				if (img.Encoding() == ColorEncoding::SRGB)
				{
					internalFormat = GL_SRGB8;
				}
				else
				{
					internalFormat = GL_RGB8;
				}
				break;
			}

			case 4:
			{
				colorFormat = GL_RGBA;

				if (img.Encoding() == ColorEncoding::SRGB)
				{
					internalFormat = GL_SRGB8_ALPHA8;
				}
				else
				{
					internalFormat = GL_RGBA8;
				}
				break;
			}

			default:
				internal::Logger::Instance().Error("Invalid image channel count \"" + std::to_string(img.Channels()) + "\" while loading texture.");
				return;
		}

		glCreateTextures(GL_TEXTURE_2D, 1, &m_Texture);
		glTextureStorage2D(m_Texture, 1, internalFormat, m_Width, m_Height);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // This is a permanent change because of the tightly packed image data. Unnecessary to set it every time.
		glTextureSubImage2D(m_Texture, 0, 0, 0, m_Width, m_Height, colorFormat, GL_UNSIGNED_BYTE, img.Data().data());

		glGenerateTextureMipmap(m_Texture);

		glTextureParameteri(m_Texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTextureParameteri(m_Texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}


	GlTexture::~GlTexture() noexcept
	{
		glDeleteTextures(1, &m_Texture);
	}


	auto GlTexture::TextureName() const noexcept -> u32
	{
		return m_Texture;
	}


	auto GlTexture::Width() const noexcept -> u32
	{
		return m_Width;
	}


	auto GlTexture::Height() const noexcept -> u32
	{
		return m_Height;
	}
}

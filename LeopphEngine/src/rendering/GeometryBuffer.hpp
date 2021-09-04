#pragma once

#include "../events/DisplayResolutionChangedEvent.hpp"
#include "../events/EventReceiver.hpp"
#include "../math/Vector.hpp"

#include <array>


namespace leopph::impl
{
	class GeometryBuffer final : public EventReceiver<DisplayResolutionChangedEvent>
	{
	public:
		GeometryBuffer();

		GeometryBuffer(const GeometryBuffer&) = delete;
		GeometryBuffer(GeometryBuffer&&) = delete;

		GeometryBuffer& operator=(const GeometryBuffer&) = delete;
		GeometryBuffer& operator=(GeometryBuffer&&) = delete;

		~GeometryBuffer() override;

		void Clear() const;
		void Bind() const;
		void Unbind() const;

		const unsigned& frameBufferName;
		const unsigned& depthBufferName;
		const unsigned& positionTextureName;
		const unsigned& normalTextureName;
		const unsigned& ambientTextureName;
		const unsigned& diffuseTextureName;
		const unsigned& specularTextureName;
		const unsigned& shineTextureName;


	private:
		enum TextureType
		{
			Position, Normal, Ambient, Diffuse, Specular, Shine
		};

		std::array<unsigned, 6> m_Textures;
		unsigned m_DepthBuffer;
		unsigned m_FrameBuffer;

		void SetUpBuffers(const Vector2& res);
		void OnEventReceived(EventParamType event) override;
	};
}

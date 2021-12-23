#pragma once

#include "../events/PointShadowResolutionEvent.hpp"
#include "../events/handling/EventReceiver.hpp"
#include "../rendering/shaders/ShaderProgram.hpp"

#include <cstddef>


namespace leopph::internal
{
	class CubeShadowMap final : EventReceiver<PointShadowResolutionEvent>
	{
		public:
			CubeShadowMap();

			CubeShadowMap(const CubeShadowMap& other) = delete;
			CubeShadowMap(CubeShadowMap&& other) = delete;

			CubeShadowMap& operator=(const CubeShadowMap& other) = delete;
			CubeShadowMap& operator=(CubeShadowMap&& other) = delete;

			~CubeShadowMap() override;

			void BindForWriting() const;
			void UnbindFromWriting() const;

			[[nodiscard]] int BindForReading(ShaderProgram& shader, int texUnit);
			void UnbindFromReading() const;

			void Clear() const;


		private:
			unsigned m_FrameBufferName;
			unsigned m_CubeMapName;
			int m_BoundTexUnit;

			void OnEventReceived(EventParamType event) override;

			void Init(std::size_t resolution);
			void Deinit() const;
	};
}
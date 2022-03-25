#pragma once

#include "SceneSwitcher.hpp"

#include <Leopph.hpp>
#include <vector>


namespace demo
{
	class TeleportGate final : public leopph::Behavior
	{
		public:
			struct PortPoint
			{
				// The Transform's position is the center of activation
				std::shared_ptr<leopph::Transform> ActivationPoint;
				// The distance the player has to be from the ActivationPoint
				// to trigger a scene change
				float Distance;
				// The scene to change to when triggered
				SceneSwitcher::Scene Scene;
			};


			explicit TeleportGate(leopph::Entity* player, std::shared_ptr<SceneSwitcher> sceneSwitcher);
			auto OnFrameUpdate() -> void override;
			auto AddPortPoint(const PortPoint& portPoint) -> void;

		private:
			leopph::Entity* m_Player;
			std::shared_ptr<SceneSwitcher> m_SceneSwitcher{nullptr};
			std::vector<PortPoint> m_PortPoints;
	};
}

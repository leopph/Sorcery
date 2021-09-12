#pragma once

#include "Leopph.hpp"

class CameraController final : public leopph::Behavior
{
public:
	explicit CameraController(leopph::Entity& owner);
	void OnFrameUpdate() override;

private:
	const float m_Speed;
	const float m_Sens;
	float lastX;
	float lastY;
};
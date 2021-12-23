#include "Component.hpp"


namespace leopph
{
	Component::Component(leopph::Entity* const entity) :
		m_Entity{entity}
	{}

	auto Component::Entity() const -> leopph::Entity*
	{
		return m_Entity;
	}
}

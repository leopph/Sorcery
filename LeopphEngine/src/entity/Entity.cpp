#include "Entity.hpp"

#include "../data/DataManager.hpp"
#include "../util/Logger.hpp"

#include <algorithm>
#include <array>
#include <cstddef>


namespace leopph
{
	auto Entity::Find(std::string const& name) -> Entity*
	{
		return internal::DataManager::Instance().FindEntity(name);
	}


	Entity::Entity(std::string name) :
		m_Name{
			[&]
			{
				if (!NameIsUnused(name))
				{
					name = GenerateUnusedName(name);
					internal::Logger::Instance().Warning("Name collision detected. Entity is being renamed to " + name + ".");
				}

				return name;
			}()
		}
	{
		auto& dataManager{internal::DataManager::Instance()};
		dataManager.RegisterEntity(this);
		m_Transform->Attach(this);
	}


	Entity::~Entity()
	{
		auto& dataManager{internal::DataManager::Instance()};

		m_Transform->Component::Owner(nullptr); // Transform is a special case because it cannot be detached.

		std::ranges::for_each(std::array{true, false}, [this, &dataManager](auto const active)
		{
			auto components = dataManager.ComponentsOfEntity(this, active);
			// Detach erases itself from the component collection, so we iterate backwards to not cause element relocation
			std::for_each(components.rbegin(), components.rend(), [](auto const component)
			{
				component->Detach();
			});
		});

		dataManager.UnregisterEntity(this);
	}


	auto Entity::AttachComponent(ComponentPtr<> const& component) -> void
	{
		component->Attach(this);
	}


	auto Entity::DetachComponent(ComponentPtr<> const& component) const -> void
	{
		auto const& logger{internal::Logger::Instance()};

		if (!component)
		{
			logger.Warning("Ignoring attempt to remove nullptr component from Entity [" + m_Name + "].");
			return;
		}

		if (component->Owner() != this)
		{
			internal::Logger::Instance().Error("Ignoring attempt to remove component at [" + std::to_string(reinterpret_cast<std::size_t>(component.get())) + "] from Entity [" + m_Name + "], because the component is not owned by the Entity.");
			return;
		}

		component->Detach();
	}


	auto Entity::DeactiveAllComponents() const -> void
	{
		for (auto& component : internal::DataManager::Instance().ComponentsOfEntity(this, true))
		{
			component->Deactivate();
		}
	}


	auto Entity::ActivateAllComponents() const -> void
	{
		for (auto& component : internal::DataManager::Instance().ComponentsOfEntity(this, false))
		{
			component->Activate();
		}
	}


	auto Entity::Components() const -> std::span<ComponentPtr<> const>
	{
		return internal::DataManager::Instance().ComponentsOfEntity(this, true);
	}


	auto Entity::GenerateUnusedName(std::string const& namePrefix) -> std::string
	{
		static std::size_t entityCounter{0};
		auto newName{namePrefix + std::to_string(entityCounter)};
		++entityCounter;
		while (!NameIsUnused(newName))
		{
			newName = namePrefix + std::to_string(entityCounter);
			++entityCounter;
		}
		return newName;
	}


	auto Entity::NameIsUnused(std::string const& name) -> bool
	{
		return !internal::DataManager::Instance().FindEntity(name);
	}
}

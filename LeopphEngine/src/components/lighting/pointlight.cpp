#include "PointLight.hpp"

#include "../../data/DataManager.hpp"


namespace leopph
{
	auto PointLight::Owner(Entity* entity) -> void
	{
		auto& dataManager = internal::DataManager::Instance();

		if (InUse())
		{
			dataManager.UnregisterActivePointLight(this);
		}

		AttenuatedLight::Owner(entity);

		if (InUse())
		{
			dataManager.RegisterActivePointLight(this);
		}
	}


	auto PointLight::Active(bool const active) -> void
	{
		auto& dataManager = internal::DataManager::Instance();

		if (InUse())
		{
			dataManager.UnregisterActivePointLight(this);
		}

		AttenuatedLight::Active(active);

		if (InUse())
		{
			dataManager.RegisterActivePointLight(this);
		}
	}


	auto PointLight::operator=(PointLight const& other) -> PointLight&
	{
		if (this == &other)
		{
			return *this;
		}

		auto& dataManager = internal::DataManager::Instance();

		if (InUse())
		{
			dataManager.UnregisterActivePointLight(this);
		}

		AttenuatedLight::operator=(other);

		if (InUse())
		{
			dataManager.RegisterActivePointLight(this);
		}

		return *this;
	}


	PointLight::~PointLight()
	{
		internal::DataManager::Instance().UnregisterActivePointLight(this);
	}
}

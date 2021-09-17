#include "AttenuatedLight.hpp"



namespace leopph::impl
{
	AttenuatedLight::AttenuatedLight(Entity& owner, const float constant, const float linear, const float quadratic, const float range) :
		Light{owner},
		m_Constant{constant},
		m_Linear{linear},
		m_Quadratic{quadratic},
		m_Range{range}
	{}


	AttenuatedLight::~AttenuatedLight() = default;


	float AttenuatedLight::Constant() const
	{
		return m_Constant;
	}


	float AttenuatedLight::Quadratic() const
	{
		return m_Quadratic;
	}


	float AttenuatedLight::Linear() const
	{
		return m_Linear;
	}


	void AttenuatedLight::Linear(const float value)
	{
		m_Linear = value;
	}


	void AttenuatedLight::Constant(const float value)
	{
		m_Constant = value;
	}


	void AttenuatedLight::Quadratic(const float value)
	{
		m_Quadratic = value;
	}


	float AttenuatedLight::Range() const
	{
		return m_Range;
	}


	void AttenuatedLight::Range(const float value)
	{
		m_Range = value;
	}
}

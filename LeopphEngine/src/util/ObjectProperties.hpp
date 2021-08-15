#pragma once

#include "../math/quaternion.h"
#include "../math/vector.h"

#include <string>

namespace leopph
{
	struct ObjectProperties
	{
		std::string name{};
		bool isStatic{ false };
		Vector3 position{};
		Quaternion rotation{};
		Vector3 scale{ 1, 1, 1 };
	};
}
#pragma once

#include "Event.hpp"

#include <cstddef>
#include <vector>


namespace leopph::internal
{
	struct DirShadowResolutionEvent : public Event
	{
		DirShadowResolutionEvent(const std::vector<std::size_t>& resolutions);

		const std::vector<std::size_t>& Resolutions;
	};
}

#pragma once

#include "../api/LeopphApi.hpp"


namespace leopph
{
	/*---------------------------------------------------------------------------
	The Time class provides access to information about frame and engine timings.
	---------------------------------------------------------------------------*/

	class LEOPPHAPI Time
	{
		public:
			/* The time it took in seconds to render the previous frame */
			static auto DeltaTime() -> float;

			/* The time that has passed since the initialization of your application in seconds */
			static auto FullTime() -> float;
	};
}

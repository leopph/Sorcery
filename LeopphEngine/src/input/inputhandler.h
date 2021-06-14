#pragma once

#include "../api/leopphapi.h"
#include "keycodes.h"
#include "keystate.h"

namespace leopph::impl
{	
	/*---------------------------------------------------------
	Middle layer between Window implementations and Input class
	to decouple implementation details.
	---------------------------------------------------------*/
	
	class LEOPPHAPI InputHandler
	{
	public:
		static void OnInputChange(KeyCode keyCode, KeyState keyState);
		static void OnInputChange(double x, double y);
		static void UpdateReleasedKeys();
	};
}
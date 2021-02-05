#pragma once

#include "vector.h"

namespace leopph::implementation
{
	// STRUCT TO REPRESENT A VERTEX
	struct Vertex
	{
		Vector3 position;
		Vector3 normal;
		Vector2 textureCoordinates;
	};
}
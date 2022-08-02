#pragma once

#include "Types.hpp"

#include <span>
#include <vector>


namespace leopph::convert::compress
{
	enum class Error
	{
		None, Inconsistency, Unknown
	};


	Error compress(std::span<u8> in, std::vector<u8>& out);
	Error uncompress(std::span<u8> in, u64 uncompressedSize, std::vector<u8>& out);
}
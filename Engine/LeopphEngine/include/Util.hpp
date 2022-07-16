#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace leopph
{
	[[nodiscard]] auto ReadFile(std::filesystem::path const& path) -> std::string;

	auto WriteFile(std::filesystem::path const& path, std::string_view contents) -> void;

	auto SplitLines(std::string_view str, std::vector<std::string>& out) -> void;
}
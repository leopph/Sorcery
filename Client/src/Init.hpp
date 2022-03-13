#pragma once

#include "SceneSwitcher.hpp"


namespace demo
{
	// Fills the passed vector with created entities
	auto InitChurchScene(SceneSwitcher::Scene thisScene, const SceneSwitcher::Scene nextScene) -> void;
	// Fills the passed vector with created entities
	auto InitCometScene(SceneSwitcher::Scene scene) -> void;
}

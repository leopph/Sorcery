#include "ProjectSettingsWindow.hpp"
#include "Renderer.hpp"

#include <imgui.h>

#include <format>


namespace leopph::editor {
std::string_view const PROJECT_SETTINGS_WINDOW_TITLE{ "Project Settings" };


auto DrawProjectSettingsWindow(bool& isOpen) -> void {
	//ImGui::SetNextWindowSizeConstraints(ImVec2{ 300, 300 }, ImVec2{ -1, -1 });

	if (!ImGui::Begin((std::string{ PROJECT_SETTINGS_WINDOW_TITLE } + "##Window").data(), &isOpen)) {
		ImGui::End();
		return;
	}

	if (!ImGui::BeginTable("ProjectSettingsWindowTable", 2)) {
		ImGui::End();
		return;
	}

	static int selectedSubMenu{ 0 };

	ImGui::TableNextColumn();
	if (ImGui::BeginChild("ProjectSettingsLeftChild")) {
		ImGui::MenuItem("Graphics");
	}
	ImGui::EndChild();

	ImGui::TableNextColumn();
	if (ImGui::BeginChild("ProjectSettingsRightChild")) {
		if (selectedSubMenu == 0) {
			ImGui::Text("Shadow Distance");
			ImGui::SameLine();

			float shadowDistance{ renderer::GetShadowDistance() };
			if (ImGui::InputFloat("##shadowDistanceInput", &shadowDistance, 0, 0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
				renderer::SetShadowDistance(shadowDistance);
			}

			ImGui::Text("Shadow Cascade Count");
			ImGui::SameLine();

			int cascadeCount{ renderer::GetCascadeCount() };
			if (ImGui::SliderInt("##cascadeCountInput", &cascadeCount, 1, renderer::GetMaxCascadeCount(), "%d", ImGuiSliderFlags_NoInput)) {
				renderer::SetCascadeCount(cascadeCount);
			}

			auto const cascadeSplits{ renderer::GetCascadeSplits() };
			auto const splitCount{ std::ssize(cascadeSplits) };

			for (int i = 0; i < splitCount; i++) {
				ImGui::Text("Split %d (percent)", i + 1);
				ImGui::SameLine();

				float cascadeSplit{ cascadeSplits[i] * 100.0f };

				if (ImGui::SliderFloat(std::format("##cascadeSplit {} input", i).data(), &cascadeSplit, 0, 100, "%.3f", ImGuiSliderFlags_NoInput)) {
					renderer::SetCascadeSplit(i, cascadeSplit / 100.0f);
				}
			}
		}
	}
	ImGui::EndChild();

	ImGui::EndTable();
	ImGui::End();
}
}

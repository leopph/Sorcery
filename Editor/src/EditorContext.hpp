#pragma once

#include <imgui.h>
#include "AssetStorage.hpp"
#include "Scene.hpp"
#include "ObjectFactoryManager.hpp"

#include <filesystem>
#include <atomic>
#include <variant>

namespace leopph::editor {
class Context {
	ImGuiIO& mImGuiIo;
	AssetStorage mResources;
	Scene* mScene{ nullptr };
	EditorObjectFactoryManager mFactoryManager{ CreateFactoryManager() };
	Object* mSelectedObject{ nullptr };

	static inline std::filesystem::path ASSET_DIR_REL{ "Assets" };
	static inline std::filesystem::path CACHE_DIR_REL{ "Cache" };
	static inline std::filesystem::path ASSET_FILE_EXT{ ".leopphasset" };

	std::filesystem::path mProjDirAbs;
	std::filesystem::path mAssetDirAbs;
	std::filesystem::path mCacheDirAbs;

	std::atomic<bool> mBusy;

public:
	explicit Context(ImGuiIO& imGuiIO);

	[[nodiscard]] auto GetImGuiIo() const noexcept -> ImGuiIO const&;
	[[nodiscard]] auto GetImGuiIo() noexcept -> ImGuiIO&;

	[[nodiscard]] auto GetResources() const noexcept -> AssetStorage const&;
	[[nodiscard]] auto GetResources() noexcept -> AssetStorage&;

	[[nodiscard]] auto GetScene() const noexcept -> Scene const*;
	[[nodiscard]] auto GetScene() noexcept -> Scene*;
	auto OpenScene(Scene& scene) -> void;

	[[nodiscard]] auto GetFactoryManager() const noexcept -> EditorObjectFactoryManager const&;
	[[nodiscard]] auto GetFactoryManager() noexcept -> EditorObjectFactoryManager&;

	[[nodiscard]] auto GetSelectedObject() const noexcept -> Object*;
	auto SetSelectedObject(Object* obj) noexcept -> void;

	[[nodiscard]] auto GetProjectDirectoryAbsolute() const noexcept -> std::filesystem::path const&;
	[[nodiscard]] auto GetAssetDirectoryAbsolute() const noexcept -> std::filesystem::path const&;
	[[nodiscard]] auto GetCacheDirectoryAbsolute() const noexcept -> std::filesystem::path const&;

	[[nodiscard]] inline static auto GetAssetFileExtension() noexcept -> std::filesystem::path const&;

	auto OpenProject(std::filesystem::path const& targetPath) -> void;

	[[nodiscard]] auto IsEditorBusy() const noexcept -> bool;

	template<typename Callable>
	auto ExecuteInBusyEditor(Callable&& callable) -> void;

	auto CreateMetaFileForRegisteredAsset(Object const& asset) const -> void;
	auto SaveRegisteredNativeAsset(NativeAsset const& asset) const -> void;
};

template<typename Callable>
auto Context::ExecuteInBusyEditor(Callable&& callable) -> void {
	std::thread{
		[this, callable] {
			bool isBusy{ false };
			while (!mBusy.compare_exchange_weak(isBusy, true)) {}

			auto const oldFlags{ mImGuiIo.ConfigFlags };
			mImGuiIo.ConfigFlags |= ImGuiConfigFlags_NoMouse;
			mImGuiIo.ConfigFlags |= ImGuiConfigFlags_NavNoCaptureKeyboard;
			std::invoke(callable);
			mImGuiIo.ConfigFlags = oldFlags;

			mBusy = false;
		}
	}.detach();
}

inline auto Context::GetAssetFileExtension() noexcept -> std::filesystem::path const& {
	return ASSET_FILE_EXT;
}
}
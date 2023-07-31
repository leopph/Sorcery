#pragma once

#include "Guid.hpp"
#include "NativeResource.hpp"

#include <filesystem>
#include <map>
#include <memory>


namespace sorcery::mage {
class ResourceDB {
  std::filesystem::path mResDirAbs;
  std::map<Guid, std::filesystem::path> mGuidToAbsPath;

  constexpr static std::string_view RESOURCE_DIR_PROJ_REL{ "Resources" };

public:
  auto Refresh() -> void;
  auto ChangeProjectDir(std::filesystem::path const& projDirAbs) -> void;
  [[nodiscard]] auto GetResourceDirectoryAbsolutePath() -> std::filesystem::path const&;

  auto CreateResource(NativeResource& res, std::filesystem::path const& targetPathResDirRel) -> void;
  auto SaveResource(NativeResource const& res) -> void;
  auto ImportResource(std::filesystem::path const& targetPathResDirRel) -> void;
  auto MoveResource(Guid const& guid, std::filesystem::path const& targetPathResDirRel) -> void;
  auto DeleteResource(Guid const& guid) -> void;
  [[nodiscard]] auto IsSavedResource(NativeResource const& res) const -> bool;

  [[nodiscard]] auto GenerateUniqueResourceDirectoryRelativePath(std::filesystem::path const& targetPathResDirRel) const -> std::filesystem::path;
};
}

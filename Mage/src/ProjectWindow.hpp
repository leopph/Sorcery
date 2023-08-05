#pragma once

#include "Application.hpp"

#include <filesystem>
#include <optional>
#include <string>


namespace sorcery::mage {
class ProjectWindow {
  struct RenameInfo {
    std::string newName;
    std::filesystem::path nodePathAbs;
  };


  struct ImportModalFileData {
    YAML::Node node;
    std::filesystem::path dstPathResDirRel;
  };


  Application* mApp;
  bool mIsOpen{true};
  std::filesystem::path mSelectedPathResDirRel; // empty if not selected
  std::optional<RenameInfo> mRenameInfo; // nullopt if not renaming
  std::vector<ImportModalFileData> mImportModalFiles{};
  bool mOpenImportModal{false};
  bool mOpenContextMenu{false};

  constexpr static std::string_view CONTEXT_MENU_ID{"ContextMenu"};

  // Returns whether the drawn subtree was modified.
  [[nodiscard]] auto DrawFilesystemTree(std::filesystem::path const& resDirAbs, std::filesystem::path const& thisPathResDirRel) noexcept -> bool;
  auto DrawContextMenu() noexcept -> void;
  auto StartRenamingSelected() noexcept -> void;

public:
  explicit ProjectWindow(Application& context);

  auto Draw() -> void;
};
}

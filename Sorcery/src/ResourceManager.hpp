#pragma once

#include "Core.hpp"
#include "Serialization.hpp"
#include "Resources/Resource.hpp"

#include <concepts>
#include <filesystem>
#include <vector>


namespace sorcery {
namespace detail {
class ResourceHandleBase {
public:
  ResourceHandleBase() = default;
  ResourceHandleBase(ResourceHandleBase const& other) = default;
  ResourceHandleBase(ResourceHandleBase&& other) noexcept = default;

  virtual ~ResourceHandleBase() = default;

  auto operator=(ResourceHandleBase const& other) -> ResourceHandleBase& = default;
  auto operator=(ResourceHandleBase&& other) noexcept -> ResourceHandleBase& = default;

  [[nodiscard]] virtual auto Get() const -> ObserverPtr<Resource> = 0;
  [[nodiscard]] virtual auto GetOrLoad() -> ObserverPtr<Resource> = 0;
};
}


constexpr inline struct NullResT {} nullres;


template<std::derived_from<Resource> ResType>
class ResourceHandle final : public detail::ResourceHandleBase {
  friend class ResourceManager;

  Guid mGuid;
  std::weak_ptr<ResType> mResource;

public:
  ResourceHandle() = default;
  ResourceHandle(ResourceHandle const& other) = default;
  ResourceHandle(ResourceHandle&& other) noexcept = default;

  ~ResourceHandle() override = default;

  auto operator=(ResourceHandle const& other) -> ResourceHandle& = default;
  auto operator=(ResourceHandle&& other) noexcept -> ResourceHandle& = default;

  auto operator=(NullResT other) -> ResourceHandle&;

  [[nodiscard]] auto operator==(ResourceHandle const& other) const -> bool;
  [[nodiscard]] auto operator==(NullResT other) const -> bool;

  [[nodiscard]] auto Get() const -> ObserverPtr<Resource> override;
  [[nodiscard]] auto GetOrLoad() -> ObserverPtr<Resource> override;
};


class ResourceManager {
  struct ResourceGuidLess {
    using is_transparent = void;
    [[nodiscard]] auto operator()(std::shared_ptr<Resource> const& lhs, std::shared_ptr<Resource> const& rhs) const noexcept -> bool;
    [[nodiscard]] auto operator()(std::shared_ptr<Resource> const& lhs, Guid const& rhs) const noexcept -> bool;
    [[nodiscard]] auto operator()(Guid const& lhs, std::shared_ptr<Resource> const& rhs) const noexcept -> bool;
  };


  std::set<std::shared_ptr<Resource>, ResourceGuidLess> mResources;
  std::map<Guid, std::filesystem::path> mGuidPathMappings;

  [[nodiscard]] auto InternalLoadResource(std::filesystem::path const& src) -> std::shared_ptr<Resource>;

public:
  constexpr static std::string_view RESOURCE_META_FILE_EXT{ ".mojo" };

  LEOPPHAPI auto LoadResource(Guid const& guid) -> ResourceHandle<Resource>;
  template<std::derived_from<Resource> ResType>
  auto Load(Guid const& guid) -> ResourceHandle<ResType>;

  LEOPPHAPI auto Unload(Guid const& guid) -> void;
  LEOPPHAPI auto UnloadAll() -> void;

  [[nodiscard]] LEOPPHAPI auto IsLoaded(Guid const& guid) const -> bool;

  LEOPPHAPI auto Add(std::shared_ptr<Resource>&& resource) -> void;

  LEOPPHAPI auto UpdateGuidPathMappings(std::map<Guid, std::filesystem::path> mappings) -> void;

  template<std::derived_from<Resource> T>
  auto FindResourcesOfType(std::vector<std::weak_ptr<T>>& out) -> void;
};


LEOPPHAPI extern ResourceManager gResourceManager;


template<std::derived_from<Resource> ResType>
auto ResourceHandle<ResType>::operator=([[maybe_unused]] NullResT other) -> ResourceHandle& {
  mResource.reset();
  mGuid = Guid::Invalid();
  return *this;
}


template<std::derived_from<Resource> ResType>
auto ResourceHandle<ResType>::operator==(ResourceHandle const& other) const -> bool {
  return mGuid == other.mGuid;
}


template<std::derived_from<Resource> ResType>
auto ResourceHandle<ResType>::operator==([[maybe_unused]] NullResT other) const -> bool {
  return !mGuid.IsValid();
}


template<std::derived_from<Resource> ResType>
auto ResourceHandle<ResType>::Get() const -> ObserverPtr<Resource> {
  return mResource.lock().get();
}


template<std::derived_from<Resource> ResType>
auto ResourceHandle<ResType>::GetOrLoad() -> ObserverPtr<Resource> {
  if (auto const currentlyReferenced{ Get() }) {
    return currentlyReferenced;
  }

  *this = gResourceManager.Load<ResType>(mGuid);
  return mResource.lock().get();
}


template<std::derived_from<Resource> ResType>
auto ResourceManager::Load(Guid const& guid) -> ResourceHandle<ResType> {
  ResourceHandle<ResType> rh;

  if (auto const it{ mGuidPathMappings.find(guid) }; it != std::end(mGuidPathMappings)) {
    auto const res{ InternalLoadResource(it->second) };

    if constexpr (!std::is_same_v<ResType, Resource>) {
      if (rttr::rttr_cast<ResType*>(res.get())) {
        rh.mResource = std::static_pointer_cast<ResType>(res);
        rh.mGuid = res->GetGuid();
      }
    } else {
      rh.mResource = res;
      rh.mGuid = res->GetGuid();
    }
  }

  return rh;
}


template<std::derived_from<Resource> T>
auto ResourceManager::FindResourcesOfType(std::vector<std::weak_ptr<T>>& out) -> void {
  for (auto const& res : mResources) {
    if (rttr::rttr_cast<T*>(res.get())) {
      out.emplace_back(std::static_pointer_cast<T>(res));
    }
  }
}
}
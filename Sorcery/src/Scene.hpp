#pragma once

#include "Entity.hpp"
#include "NativeAsset.hpp"

#include "YamlInclude.hpp"

#include <string>
#include <memory>
#include <vector>
#include <span>


namespace sorcery {
class Scene : public NativeAsset {
  RTTR_ENABLE(NativeAsset)
  std::vector<std::unique_ptr<Entity>> mEntities;
  YAML::Node mYamlData;

public:
  LEOPPHAPI static Type const SerializationType;
  LEOPPHAPI auto GetSerializationType() const -> Type override;

  LEOPPHAPI auto CreateEntity() -> Entity&;
  LEOPPHAPI auto DestroyEntity(Entity const& entityToRemove) -> void;

  [[nodiscard]] LEOPPHAPI auto GetEntities() const noexcept -> std::span<std::unique_ptr<Entity> const>;

  LEOPPHAPI auto Serialize(std::vector<std::uint8_t>& out) const noexcept -> void override;
  LEOPPHAPI auto Deserialize(std::span<std::uint8_t const> bytes) -> void;

  LEOPPHAPI auto Save() -> void;
  LEOPPHAPI auto Load(ObjectInstantiatorManager const& manager) -> void;

  LEOPPHAPI auto Clear() -> void;
};
}
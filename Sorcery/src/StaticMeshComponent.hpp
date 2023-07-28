#pragma once

#include "Component.hpp"
#include "Resources/Material.hpp"
#include "Resources/Mesh.hpp"

#include <span>
#include <vector>


namespace sorcery {
class StaticMeshComponent : public Component {
  RTTR_ENABLE(Component)
  std::vector<ObserverPtr<Material>> mMaterials;
  ObserverPtr<Mesh> mMesh;

  auto AdjustMaterialListForMesh() -> void;

public:
  LEOPPHAPI StaticMeshComponent();
  ~StaticMeshComponent() override;

  [[nodiscard]] LEOPPHAPI auto GetMaterials() const noexcept -> std::span<ObserverPtr<Material> const>;
  LEOPPHAPI auto SetMaterials(std::vector<ObserverPtr<Material>> materials) -> void;
  LEOPPHAPI auto ReplaceMaterial(int idx, Material& mtl) -> void;

  [[nodiscard]] LEOPPHAPI auto GetMesh() const noexcept -> Mesh&;
  LEOPPHAPI auto SetMesh(Mesh& mesh) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetSerializationType() const -> Type override;
  LEOPPHAPI static Type const SerializationType;

  [[nodiscard]] LEOPPHAPI auto CalculateBounds() const noexcept -> AABB;
};
}

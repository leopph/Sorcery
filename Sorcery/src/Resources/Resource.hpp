#pragma once

#include "../Object.hpp"
#include "../Guid.hpp"


namespace sorcery {
class Resource : public Object {
  RTTR_ENABLE(Object)
  Guid mGuid{Guid::Generate()};

public:
  enum class Category : std::int32_t {
    Texture  = 0,
    Mesh     = 1,
    Scene    = 2,
    Material = 3
  };


  [[nodiscard]] LEOPPHAPI auto GetGuid() const -> Guid const&;
  LEOPPHAPI auto SetGuid(Guid const& guid) -> void;
};
}

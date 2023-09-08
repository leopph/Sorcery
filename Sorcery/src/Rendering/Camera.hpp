#pragma once

#include "../Core.hpp"
#include "../Math.hpp"

#include <cstdint>


namespace sorcery {
class Camera {
public:
  enum class Type : std::uint8_t {
    Perspective  = 0,
    Orthographic = 1
  };

private:
  constexpr static float MINIMUM_PERSPECTIVE_NEAR_CLIP_PLANE{0.03f};
  constexpr static float MINIMUM_PERSPECTIVE_FAR_CLIP_PLANE_OFFSET{0.1f};
  constexpr static float MINIMUM_PERSPECTIVE_HORIZONTAL_FOV{5.0f};
  constexpr static float MINIMUM_ORTHOGRAPHIC_HORIZONTAL_SIZE{0.1f};

  float mNear{MINIMUM_PERSPECTIVE_NEAR_CLIP_PLANE};
  float mFar{100.f};
  float mOrthoSizeHoriz{10};
  float mPerspFovHorizDeg{90};
  Type mType{Type::Perspective};

public:
  [[nodiscard]] virtual auto GetPosition() const noexcept -> Vector3 = 0;
  [[nodiscard]] virtual auto GetRightAxis() const noexcept -> Vector3 = 0;
  [[nodiscard]] virtual auto GetUpAxis() const noexcept -> Vector3 = 0;
  [[nodiscard]] virtual auto GetForwardAxis() const noexcept -> Vector3 = 0;

  [[nodiscard]] LEOPPHAPI auto GetNearClipPlane() const noexcept -> float;
  LEOPPHAPI auto SetNearClipPlane(float nearClipPlane) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetFarClipPlane() const noexcept -> float;
  LEOPPHAPI auto SetFarClipPlane(float farClipPlane) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetType() const noexcept -> Type;
  LEOPPHAPI auto SetType(Type type) noexcept -> void;

  [[nodiscard]] LEOPPHAPI auto GetHorizontalPerspectiveFov() const -> float;
  LEOPPHAPI auto SetHorizontalPerspectiveFov(float degrees) -> void;

  [[nodiscard]] LEOPPHAPI auto GetHorizontalOrthographicSize() const -> float;
  LEOPPHAPI auto SetHorizontalOrthographicSize(float size) -> void;

  [[nodiscard]] LEOPPHAPI auto CalculateViewMatrix() const noexcept -> Matrix4;
  [[nodiscard]] LEOPPHAPI auto CalculateProjectionMatrix(float aspectRatio) const noexcept -> Matrix4;

  [[nodiscard]] LEOPPHAPI static auto HorizontalPerspectiveFovToVertical(float fovDegrees, float aspectRatio) noexcept -> float;
  [[nodiscard]] LEOPPHAPI static auto VerticalPerspectiveFovToHorizontal(float fovDegrees, float aspectRatio) noexcept -> float;

  Camera() = default;
  Camera(Camera const& other) = default;
  Camera(Camera&& other) noexcept = default;

  auto operator=(Camera const& other) -> Camera& = default;
  auto operator=(Camera&& other) noexcept -> Camera& = default;

  virtual ~Camera() = default;
};
}
#include "Color.hpp"

#include <algorithm>


namespace sorcery {
Color::Color(u8 const red, u8 const green, u8 const blue, u8 const alpha) :
  red{ red },
  green{ green },
  blue{ blue },
  alpha{ alpha } {}


Color::Color(Vector4 const& vec) :
  red{ static_cast<u8>(std::clamp(vec[0], 0.f, 1.f) * 255) },
  green{ static_cast<u8>(std::clamp(vec[1], 0.f, 1.f) * 255) },
  blue{ static_cast<u8>(std::clamp(vec[2], 0.f, 1.f) * 255) },
  alpha{ static_cast<u8>(std::clamp(vec[3], 0.f, 1.f) * 255) } {}


Color::operator Vector4() const {
  return Vector4
  {
    static_cast<f32>(red) / 255.f,
    static_cast<f32>(green) / 255.f,
    static_cast<f32>(blue) / 255.f,
    static_cast<f32>(alpha) / 255.f
  };
}


Color Color::Black() {
  return Color{ 0, 0, 0, 255 };
}


Color Color::Red() {
  return Color{ 255, 0, 0, 255 };
}


Color Color::Green() {
  return Color{ 0, 255, 0, 255 };
}


Color Color::Blue() {
  return Color{ 0, 0, 255, 255 };
}


Color Color::Cyan() {
  return Color{ 0, 255, 255, 255 };
}


Color Color::Magenta() {
  return Color{ 255, 0, 255, 255 };
}


Color Color::Yellow() {
  return Color{ 255, 255, 0, 255 };
}


Color Color::White() {
  return Color{ 255, 255, 255, 255 };
}
}
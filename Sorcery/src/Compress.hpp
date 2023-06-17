#pragma once

#include "Core.hpp"

#include <span>
#include <vector>


namespace sorcery {
enum class CompressionError {
  None,
  Inconsistency,
  Unknown
};


auto Compress(std::span<u8> in, std::vector<u8>& out) -> CompressionError;
auto Uncompress(std::span<u8 const> in, u64 uncompressedSize, std::vector<u8>& out) -> CompressionError;
}
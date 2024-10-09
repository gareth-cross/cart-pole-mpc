// Copyright 2024 Gareth Cross.
#pragma once
#include <array>
#include <cstdint>
#include <unordered_map>

namespace pendulum {

// Types of variables that exist in our optimization problem.
enum class KeyType : std::uint32_t {
  // Position of the base.
  B_X,
  // Angle of the pole.
  THETA_1,
  // Velocity of the base.
  B_X_DOT,
  // Angular rate of the pole.
  THETA_1_DOT,
  // Control input.
  U,
};

// Pair together a type with an index in the window.
class Key {
 public:
  constexpr Key(KeyType type, std::size_t index)
      : type_(type), index_(static_cast<std::uint32_t>(index)) {}

  constexpr bool operator==(const Key other) const noexcept {
    return type_ == other.type_ && index_ == other.index_;
  }

  constexpr bool operator!=(const Key other) const noexcept { return !operator==(other); }

 private:
  KeyType type_;
  std::uint32_t index_;
};

}  // namespace pendulum

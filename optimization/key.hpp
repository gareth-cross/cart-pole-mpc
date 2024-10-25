// Copyright (c) 2024 Gareth Cross.
#pragma once
#include <cstdint>

namespace pendulum {

// Types of variables that exist in our optimization problem.
enum class KeyType : std::uint32_t {
  // Position of the base.
  B_X = 0,
  // Angle of the pole.
  THETA_1,
  // Velocity of the base.
  B_X_DOT,
  // Angular rate of the pole.
  THETA_1_DOT,
  // Control input.
  U,
};

}  // namespace pendulum

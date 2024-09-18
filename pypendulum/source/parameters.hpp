// Copyright 2024 Gareth Cross.
#pragma once

namespace pendulum {

// Struct used to pass parameters back and forth from Python to the dynamics model.
struct PendulumParams {
  double m_b;
  double m_1;
  double m_2;
  double l_1;
  double l_2;
  double g;

  PendulumParams() noexcept = default;

  constexpr PendulumParams(double m_b, double m_1, double m_2, double l_1, double l_2,
                           double g) noexcept
      : m_b(m_b), m_1(m_1), m_2(m_2), l_1(l_1), l_2(l_2), g(g) {}
};

}  // namespace pendulum

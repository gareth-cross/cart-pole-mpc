// Copyright 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

namespace pendulum {

// Parameters of the single-pole system.
struct SingleCartPoleParams {
  double m_b;
  double m_1;
  double l_1;
  double g;

  SingleCartPoleParams() noexcept = default;

  constexpr SingleCartPoleParams(double m_b, double m_1, double l_1, double g) noexcept
      : m_b(m_b), m_1(m_1), l_1(l_1), g(g) {}
};

// Struct used to pass parameters back and forth from Python to the dynamics model.
struct DoubleCartPoleParams {
  double m_b;
  double m_1;
  double m_2;
  double l_1;
  double l_2;
  double g;

  DoubleCartPoleParams() noexcept = default;

  constexpr DoubleCartPoleParams(double m_b, double m_1, double m_2, double l_1, double l_2,
                                 double g) noexcept
      : m_b(m_b), m_1(m_1), m_2(m_2), l_1(l_1), l_2(l_2), g(g) {}
};

// State of the single cart-pole system.
struct SingleCartPoleState {
  double b_x;
  double th_1;
  double b_x_dot;
  double th_1_dot;

  constexpr SingleCartPoleState(double b_x, double th_1, double b_x_dot, double th_1_dot) noexcept
      : b_x(b_x), th_1(th_1), b_x_dot(b_x_dot), th_1_dot(th_1_dot) {}

  explicit SingleCartPoleState(const Eigen::Vector4d& x) noexcept
      : SingleCartPoleState(x[0], x[1], x[2], x[3]) {}

  Eigen::Vector4d ToVector() const noexcept { return {b_x, th_1, b_x_dot, th_1_dot}; }
};

}  // namespace pendulum

// Copyright 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

namespace pendulum {

// Parameters of the single-pole system.
struct SingleCartPoleParams {
  // Mass of the base (kg).
  double m_b;
  // Mass of the object on the pole (kg).
  double m_1;
  // Length of the pole (meters).
  double l_1;
  // Gravity (m/s^2).
  double g;
  // Friction at the base.
  double mu_b;
  // Cutoff velocity of the smoothed Coulomb model (m/s).
  double v_mu_b;
  // Drag coefficient on the pole mass (W / (m/s)^3).
  double c_d_1;
  // Position of the bumper springs (meters).
  double x_s;
  // Spring constant of the bumper sprints (N/m).
  double k_s;

  SingleCartPoleParams() noexcept = default;

  constexpr SingleCartPoleParams(double m_b, double m_1, double l_1, double g, double mu_b,
                                 double v_mu_b, double c_d_1, double x_s, double k_s) noexcept
      : m_b(m_b),
        m_1(m_1),
        l_1(l_1),
        g(g),
        mu_b(mu_b),
        v_mu_b(v_mu_b),
        c_d_1(c_d_1),
        x_s(x_s),
        k_s(k_s) {}
};

// State of the single cart-pole system.
struct SingleCartPoleState {
  // Base position on x-axis, b_x(t).
  double b_x;
  // Angle of the pole, θ(t).
  double th_1;
  // Derivative of b_x(t).
  double b_x_dot;
  // Derivative of θ(t).
  double th_1_dot;

  // Default-constructible to support serialization.
  SingleCartPoleState() noexcept = default;

  constexpr SingleCartPoleState(double b_x, double th_1, double b_x_dot, double th_1_dot) noexcept
      : b_x(b_x), th_1(th_1), b_x_dot(b_x_dot), th_1_dot(th_1_dot) {}

  explicit SingleCartPoleState(const Eigen::Vector4d& x) noexcept
      : SingleCartPoleState(x[0], x[1], x[2], x[3]) {}

  Eigen::Vector4d ToVector() const noexcept { return {b_x, th_1, b_x_dot, th_1_dot}; }
};

// For passing vectors from JavaScript to C++.
struct Vector2 {
  double x;
  double y;
};

}  // namespace pendulum

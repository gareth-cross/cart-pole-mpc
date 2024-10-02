// Unit test integration code.
// Copyright 2024 Gareth Cross.

#include <gtest/gtest.h>

#include "integration.hpp"
#include "single_pendulum_dynamics.hpp"

namespace pendulum {

template <typename Scalar, typename Function>
auto numerical_derivative(const Scalar dx, Function func) -> decltype(func(dx)) {
  using result_type = decltype(func(dx));
  const Scalar dx2 = dx * 2;
  const Scalar dx3 = dx * 3;
  const result_type c1 = func(dx) - func(-dx);
  const result_type c2 = func(dx2) - func(-dx2);
  const result_type c3 = func(dx3) - func(-dx3);
  return (c1 * 45 - c2 * 9 + c3) / (60 * dx);
}

template <int XDim, typename Function>
auto numerical_jacobian(const Eigen::Matrix<double, XDim, 1>& x, Function func,
                        const double h = 0.01) {
  // Compute the output expression at the linearization point.
  const auto y_0 = func(x);
  constexpr int YDim = std::decay_t<decltype(y_0)>::RowsAtCompileTime;

  // Storage for jacobian + tangent space delta:
  Eigen::Matrix<double, YDim, XDim> J;
  Eigen::Matrix<double, XDim, 1> delta;

  for (int j = 0; j < XDim; ++j) {
    // Take derivative wrt dimension `j` of X
    J.col(j) = numerical_derivative(h, [&](double dx) -> Eigen::Matrix<double, YDim, 1> {
      delta.setZero();
      delta[j] = dx;
      const auto y = func(x + delta);
      return y - y_0;
    });
  }
  return J;
}

// Test numerical integration of the single pendulum dynamics model.
TEST(IntegrationTest, TestSingle) {
  const Eigen::Matrix<double, 4, 1> x{1.2, 0.7, 0.4, -0.15};
  constexpr double u = 0.1;
  constexpr double dt = 0.01;

  constexpr PendulumParams params{1.0, 0.1, 0.0, 0.25, 0.0, 9.81};

  const auto [_, f_D_x, f_D_u] = runge_kutta_4th_order<4>(
      x, u, dt,
      [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u,
          Eigen::Matrix<double, 4, 4>& x_dot_D_x, Eigen::Matrix<double, 4, 1>& x_dot_D_u) {
        Eigen::Matrix<double, 4, 1> x_dot;
        gen::single_pendulum_dynamics(params, x_updated, u, x_dot, x_dot_D_x, x_dot_D_u);
        return x_dot;
      });

  const auto f_D_x_numerical =
      numerical_jacobian(x, [&](const Eigen::Matrix<double, 4, 1>& x_perturb) {
        return std::get<0>(runge_kutta_4th_order<4>(
            x_perturb, u, dt,
            [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u,
                Eigen::Matrix<double, 4, 4>& x_dot_D_x, Eigen::Matrix<double, 4, 1>& x_dot_D_u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              gen::single_pendulum_dynamics(params, x_updated, u, x_dot, x_dot_D_x, x_dot_D_u);
              return x_dot;
            }));
      });

  const auto f_D_u_numerical = numerical_jacobian(
      Eigen::Matrix<double, 1, 1>{u}, [&](const Eigen::Matrix<double, 1, 1>& u_perturb) {
        return std::get<0>(runge_kutta_4th_order<4>(
            x, u_perturb[0], dt,
            [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u,
                Eigen::Matrix<double, 4, 4>& x_dot_D_x, Eigen::Matrix<double, 4, 1>& x_dot_D_u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              gen::single_pendulum_dynamics(params, x_updated, u, x_dot, x_dot_D_x, x_dot_D_u);
              return x_dot;
            }));
      });

  ASSERT_LT((f_D_x - f_D_x_numerical).norm(), 1.0e-12);
  ASSERT_LT((f_D_u - f_D_u_numerical).norm(), 1.0e-12);
}

}  // namespace pendulum

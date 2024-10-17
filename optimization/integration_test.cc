// Copyright (c) 2024 Gareth Cross.
#include <fmt/ostream.h>
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

// Test jacobians of the numerical integration of the single pendulum dynamics model.
TEST(IntegrationTest, TestDerivatives) {
  const Eigen::Matrix<double, 4, 1> x{1.2, 0.7, 0.4, -0.15};
  constexpr double u = 0.1;
  constexpr double dt = 0.01;
  constexpr SingleCartPoleParams params{1.0, 0.1, 0.25, 9.81, 0.0, 0.1, 0.0};
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();

  const auto [_, f_D_x, f_D_u] = runge_kutta_4th_order<4>(
      x, u, dt,
      [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u,
          Eigen::Matrix<double, 4, 4>& x_dot_D_x, Eigen::Matrix<double, 4, 1>& x_dot_D_u) {
        Eigen::Matrix<double, 4, 1> x_dot;
        gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, x_dot_D_x,
                                      x_dot_D_u);
        return x_dot;
      });

  const auto f_D_x_numerical =
      numerical_jacobian(x, [&](const Eigen::Matrix<double, 4, 1>& x_perturb) {
        return runge_kutta_4th_order_no_jacobians<4>(
            x_perturb, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, nullptr,
                                            nullptr);
              return x_dot;
            });
      });

  const auto f_D_u_numerical = numerical_jacobian(
      Eigen::Matrix<double, 1, 1>{u}, [&](const Eigen::Matrix<double, 1, 1>& u_perturb) {
        return runge_kutta_4th_order_no_jacobians<4>(
            x, u_perturb[0], dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, nullptr,
                                            nullptr);
              return x_dot;
            });
      });

  ASSERT_LT((f_D_x - f_D_x_numerical).norm(), 1.0e-12);
  ASSERT_LT((f_D_u - f_D_u_numerical).norm(), 1.0e-12);
}

TEST(IntegrationTest, TestFrictionDissipation) {
  // Start with pendulum level:
  const Eigen::Matrix<double, 4, 1> x0 = SingleCartPoleState(0.0, 0.0, 0.0, 0.0).ToVector();
  constexpr double u = 0.0;
  constexpr double dt = 0.01;
  constexpr SingleCartPoleParams params{1.0, 0.5, 0.4, 9.81, 0.5, 0.1, 0.0};
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  constexpr std::size_t num_iterations = 20000;

  // Integrate and check that velocities go to zero:
  Eigen::Vector4d x = x0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    x = runge_kutta_4th_order_no_jacobians<4>(
        x, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, nullptr, nullptr);
          return x_dot;
        });
  }

  const SingleCartPoleState x_final{x};
  EXPECT_NEAR(0.0, x_final.b_x_dot, 1.0e-6);
  EXPECT_NEAR(0.0, x_final.th_1_dot, 1.0e-4);
}

TEST(IntegrationTest, TestDragDissipation) {
  const Eigen::Matrix<double, 4, 1> x0 = SingleCartPoleState(0.0, -M_PI, 0.0, 0.0).ToVector();
  constexpr double u = 0.0;
  constexpr double dt = 0.01;
  constexpr SingleCartPoleParams params{0.8, 0.1, 0.4, 9.81, 0.01, 0.1, 5.0};
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  constexpr std::size_t num_iterations = 10000;

  // Integrate and check that velocities go to zero:
  Eigen::Vector4d x = x0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    x = runge_kutta_4th_order_no_jacobians<4>(
        x, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, nullptr, nullptr);
          return x_dot;
        });
  }

  const SingleCartPoleState x_final{x};
  EXPECT_NEAR(0.0, x_final.b_x_dot, 1.0e-6);
  EXPECT_NEAR(0.0, x_final.th_1_dot, 3.0e-5);
}

TEST(IntegrationTest, TestExternalForceSymmetry) {
  // Start with pendulum hanging down, stationary:
  const Eigen::Matrix<double, 4, 1> x0 = SingleCartPoleState(0.0, -M_PI / 2, 0.0, 0.0).ToVector();
  constexpr double u = 0.0;
  constexpr double dt = 0.001;
  constexpr SingleCartPoleParams params{1.0, 0.1, 0.25, 9.81, 0.1, 0.1, 0.0};

  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();

  // Apply an external force from the left
  constexpr std::size_t num_iterations = 3000;
  constexpr std::size_t force_duration = 500;
  constexpr double force_magnitude = 5.0;

  Eigen::Vector4d x_left = x0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    x_left = runge_kutta_4th_order_no_jacobians<4>(
        x_left, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          gen::single_pendulum_dynamics(
              params, x_updated, u,
              i < force_duration ? Eigen::Vector2d{force_magnitude, 0.0} : Eigen::Vector2d::Zero(),
              zero, x_dot, nullptr, nullptr);
          return x_dot;
        });
  }

  // Apply a force from the right:
  Eigen::Vector4d x_right = x0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    x_right = runge_kutta_4th_order_no_jacobians<4>(
        x_right, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          gen::single_pendulum_dynamics(
              params, x_updated, u,
              i < force_duration ? Eigen::Vector2d{-force_magnitude, 0.0} : Eigen::Vector2d::Zero(),
              zero, x_dot, nullptr, nullptr);
          return x_dot;
        });
  }

  const SingleCartPoleState x_left_final(x_left);
  const SingleCartPoleState x_right_final(x_right);

  EXPECT_GT(x_left_final.b_x, 0);
  EXPECT_LT(x_right_final.b_x, 0);

  // The initial conditions and model are symmetric, so these should match:
  EXPECT_NEAR(x_left_final.b_x, -x_right_final.b_x, 1.0e-12);
  EXPECT_NEAR(x_left_final.b_x_dot, -x_right_final.b_x_dot, 1.0e-12);
  EXPECT_NEAR(-M_PI / 2 - x_left_final.th_1, x_right_final.th_1 - -M_PI / 2, 1.0e-12);
  EXPECT_NEAR(x_left_final.th_1_dot, -x_right_final.th_1_dot, 1.0e-12);
}

}  // namespace pendulum

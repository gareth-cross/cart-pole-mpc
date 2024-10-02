// Copyright 2024 Gareth Cross.
#include <numeric>

#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "integration.hpp"
#include "single_pendulum_dynamics.hpp"
#include "single_pendulum_energy.hpp"

#include "mini_opt/assertions.hpp"
#include "mini_opt/logging.hpp"
#include "mini_opt/nonlinear.hpp"
#include "mini_opt/residual.hpp"

namespace pendulum {

TEST(OptimizationTest, TestPendulumSystem) {
  // Length of the planning window, and control rate:
  constexpr std::size_t window_len = 200;
  constexpr double dt = 0.01;

  // Parameters of the pendulum system
  constexpr PendulumParams params{10.0, 0.1, 0.1, 0.25, 0.25, 9.81};

  // The initial state of the system:
  const Eigen::Vector4d x0 = (Eigen::Vector4d() << 0.0, M_PI / 4, 0.0, 0.0).finished();

  // Indices of all the control inputs:
  std::vector<int> indices(window_len);
  std::iota(indices.begin(), indices.end(), 0);

  mini_opt::Residual<2, Eigen::Dynamic> end_state_equality(
      indices,
      [window_len, x0, params](
          const Eigen::VectorXd& u_controls,
          Eigen::Matrix<double, 2, Eigen::Dynamic>* const J_out) -> Eigen::Vector2d {
        F_ASSERT_EQ(window_len, u_controls.size());

        // We need to integrate the state forward and compute the jacobian wrt the control state
        // for each moment in time.
        std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x(window_len);
        std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u(window_len);

        Eigen::Vector4d x = x0;
        for (std::size_t i = 0; i < window_len; ++i) {
          // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
          // input `u`.
          std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
              x, u_controls[i], dt,
              [&params](const auto& x_updated, const double u, auto&& x_dot_D_x, auto&& x_dot_D_u) {
                Eigen::Matrix<double, 4, 1> x_dot;
                gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
                                              std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                              std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
                return x_dot;
              });
        }

        // We have two end-state equality constraints:
        // - Position of the pendulum is pi/2.
        // - Kinetic energy of the system is zero.
        Eigen::Matrix<double, 2, 4> cost_D_x;
        cost_D_x.setZero();
        cost_D_x(0, 1) = 1.0;

        double kinetic_energy, potential_energy;
        gen::single_pendulum_energy(params, x, kinetic_energy, potential_energy,
                                    cost_D_x.block<1, 4>(1, 0), nullptr);

        // Go back and compute jacobians:
        if (J_out) {
          F_ASSERT_EQ(window_len, J_out->cols());
          for (int i = static_cast<int>(window_len) - 1; i >= 0; --i) {
            // Chain rule: final_state_D_state[i+1] * state[i+1]_D_u[i]
            J_out->block<2, 1>(0, i).noalias() = cost_D_x * x_new_D_u[i];
            // Chain rule backwards in time to the previous step:
            cost_D_x = (cost_D_x * x_new_D_x[i]).eval();
          }
        }

        // fmt::print("x[1] = {}, kinetic_energy = {}\n", x[1], kinetic_energy);

        if (J_out) {
          // J_out->bottomRows<1>().setZero();
        }
        return {x[1] - M_PI / 2, kinetic_energy};
      });

  //
  mini_opt::Problem problem{};
  problem.dimension = static_cast<int>(indices.size());
  problem.equality_constraints.emplace_back(
      new mini_opt::Residual<2, Eigen::Dynamic>(end_state_equality));

  for (std::size_t i = 0; i + 1 < indices.size(); ++i) {
    std::array<int, 2> indices = {static_cast<int>(i), static_cast<int>(i + 1)};
    auto function = [](const Eigen::Vector2d& u,
                       Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      const double weight = 0.01;
      if (J_out) {
        J_out->operator()(0, 0) = weight;
        J_out->operator()(0, 1) = -weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
    };
    problem.costs.emplace_back(new mini_opt::Residual<1, 2>(indices, function));
  }

  mini_opt::ConstrainedNonlinearLeastSquares nls(&problem);
  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = 50;
  p.relative_exit_tol = 0.0;

  mini_opt::Logger logger{};
  nls.SetLoggingCallback(std::bind(&mini_opt::Logger::NonlinearSolverCallback, &logger,
                                   std::placeholders::_1, std::placeholders::_2));

  Eigen::VectorXd u_params = Eigen::VectorXd::Zero(indices.size());
  const mini_opt::NLSSolverOutputs outputs = nls.Solve(p, u_params);

  std::cout << fmt::format("Termination state: {}\n{}\n", fmt::streamed(outputs.termination_state),
                           logger.GetString());

  std::cout << nls.variables().transpose().format(
                   Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]"))
            << std::endl;
}

}  // namespace pendulum

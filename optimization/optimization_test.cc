// Copyright 2024 Gareth Cross.
#include <chrono>
#include <numeric>

#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "integration.hpp"
#include "optimization.hpp"
#include "simulator.hpp"
#include "single_pendulum_dynamics.hpp"

#include "mini_opt/assertions.hpp"
#include "mini_opt/nonlinear.hpp"
#include "mini_opt/residual.hpp"

namespace pendulum {

TEST(OptimizationTest, TestPendulumSystem) {
  // Length of the planning window, and control rate:
  constexpr std::size_t window_len = 300;
  constexpr double dt = 0.01;

  // Parameters of the pendulum system
  constexpr SingleCartPoleParams params{1.0, 0.1, 0.25, 9.81};

  // The initial state of the system:
  const Eigen::Vector4d x0 = (Eigen::Vector4d() << 0.0, M_PI / 4.0, 0.0, 0.0).finished();

  // Indices of all the control inputs:
  std::vector<int> indices(window_len);
  std::iota(indices.begin(), indices.end(), 0);

  mini_opt::Residual<3, Eigen::Dynamic> end_state_equality(
      indices,
      [window_len, x0, params](
          const Eigen::VectorXd& u_controls,
          Eigen::Matrix<double, 3, Eigen::Dynamic>* const J_out) -> Eigen::Vector3d {
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
        Eigen::Matrix<double, 3, 4> cost_D_x;
        cost_D_x.setZero();
        cost_D_x(0, 1) = 1.0;
        cost_D_x(1, 2) = 1.0;
        cost_D_x(2, 3) = 1.0;

        // double kinetic_energy, potential_energy;
        // gen::single_pendulum_energy(params, x, kinetic_energy, potential_energy,
        //                             cost_D_x.block<1, 4>(1, 0), nullptr);

        // Go back and compute jacobians:
        if (J_out) {
          F_ASSERT_EQ(window_len, J_out->cols());
          for (int i = static_cast<int>(window_len) - 1; i >= 0; --i) {
            // Chain rule: final_state_D_state[i+1] * state[i+1]_D_u[i]
            J_out->block<3, 1>(0, i).noalias() = cost_D_x * x_new_D_u[i];
            // Chain rule backwards in time to the previous step:
            cost_D_x = (cost_D_x * x_new_D_x[i]).eval();
          }
        }

        return {x[1] - M_PI / 2, x[2], x[3]};
      });

  //
  mini_opt::Problem problem{};
  problem.dimension = static_cast<int>(indices.size());
  problem.equality_constraints.emplace_back(
      new mini_opt::Residual<3, Eigen::Dynamic>(end_state_equality));

  for (std::size_t i = 0; i + 1 < indices.size(); ++i) {
    std::array<int, 2> indices = {static_cast<int>(i), static_cast<int>(i + 1)};
    auto function = [](const Eigen::Vector2d& u,
                       Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      constexpr double weight = 0.01;
      if (J_out) {
        J_out->operator()(0, 0) = weight;
        J_out->operator()(0, 1) = -weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
    };
    problem.costs.emplace_back(new mini_opt::Residual<1, 2>(indices, function));
  }

  // place costs on the trajectory
  // for (std::size_t i = 1; i < indices.size(); ++i) {
  //   std::vector<int> indices_up_to_i(i);
  //   std::iota(indices_up_to_i.begin(), indices_up_to_i.end(), 0);

  //   const std::size_t num_states = i;
  //   mini_opt::Residual<2, Eigen::Dynamic> angle_penalty(
  //       indices_up_to_i,
  //       [num_states, x0, params](
  //           const Eigen::VectorXd& u_controls,
  //           Eigen::Matrix<double, 2, Eigen::Dynamic>* const J_out) -> Eigen::Matrix<double, 2, 1>
  //           {
  //         F_ASSERT_EQ(num_states, static_cast<std::size_t>(u_controls.size()));

  //         // We need to integrate the state forward and compute the jacobian wrt the control
  //         state
  //         // for each moment in time.
  //         std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x(num_states);
  //         std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u(num_states);

  //         Eigen::Vector4d x = x0;
  //         for (std::size_t i = 0; i < num_states; ++i) {
  //           // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
  //           // input `u`.
  //           std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
  //               x, u_controls[i], dt,
  //               [&params](const auto& x_updated, const double u, auto&& x_dot_D_x,
  //                         auto&& x_dot_D_u) {
  //                 Eigen::Matrix<double, 4, 1> x_dot;
  //                 gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
  //                                               std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
  //                                               std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
  //                 return x_dot;
  //               });
  //         }

  //         // Penalize position of the pendulum:
  //         // - Position of the pendulum is pi/2.
  //         const double weight = static_cast<double>(num_states) / window_len;
  //         Eigen::Matrix<double, 2, 4> cost_D_x;
  //         cost_D_x.setZero();
  //         cost_D_x(0, 0) = weight;
  //         cost_D_x(1, 1) = weight;

  //         // double kinetic_energy, potential_energy;
  //         // gen::single_pendulum_energy(params, x, kinetic_energy, potential_energy,
  //         //                             cost_D_x.block<1, 4>(1, 0), nullptr);

  //         // Go back and compute jacobians:
  //         if (J_out) {
  //           F_ASSERT_EQ(static_cast<int>(num_states), J_out->cols());
  //           for (int i = static_cast<int>(num_states) - 1; i >= 0; --i) {
  //             // Chain rule: final_state_D_state[i+1] * state[i+1]_D_u[i]
  //             J_out->middleCols<1>(i).noalias() = cost_D_x * x_new_D_u[i];
  //             // Chain rule backwards in time to the previous step:
  //             cost_D_x = (cost_D_x * x_new_D_x[i]).eval();
  //           }
  //         }
  //         return Eigen::Matrix<double, 2, 1>{weight * x[0], weight * (x[1] - M_PI / 2)};
  //       });
  //   problem.costs.emplace_back(new mini_opt::Residual<2, Eigen::Dynamic>(angle_penalty));
  // }

  mini_opt::ConstrainedNonlinearLeastSquares nls(&problem);
  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = 300;
  p.relative_exit_tol = 0.0;
  p.equality_penalty_scale_factor = 1.01;
  p.max_qp_iterations = 1;
  // p.line_search_strategy = mini_opt::LineSearchStrategy::ARMIJO_BACKTRACK;
  // p.armijo_search_tau = 0.25;
  p.max_line_search_iterations = 5;

  Eigen::VectorXd u_params = Eigen::VectorXd::Zero(indices.size());
  const mini_opt::NLSSolverOutputs outputs = nls.Solve(p, u_params);

  std::cout << fmt::format("Termination state: {}\n{}\n", fmt::streamed(outputs.termination_state),
                           outputs.ToString(true));

  std::cout << nls.variables().transpose().format(
                   Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]"))
            << std::endl;
}

TEST(OptimizationTest, TestCartPoleMultipleShooting) {
  // Length of the planning window, and control rate:
  constexpr std::size_t window_len = 100;
  constexpr double dt = 0.01;

  // Parameters of the pendulum system
  constexpr SingleCartPoleParams params{1.0, 0.1, 0.25, 9.81};

  // The initial state of the system:
  const Eigen::Vector4d x0 = (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.0).finished();

  // Total # of parameters:
  constexpr std::size_t state_multiplier = 5;
  static_assert(window_len % state_multiplier == 0);

  // Number of states, plus one more to bound on both sides:
  constexpr std::size_t num_states = (window_len / state_multiplier) + 1;

  mini_opt::Problem problem{};
  problem.dimension = static_cast<int>(num_states * 4 + window_len);

  for (std::size_t i = 0; i + 1 < num_states; ++i) {
    std::vector<int> indices;
    auto it = std::back_inserter(indices);
    for (int k = 0; k < 4; ++k) {
      *(it++) = static_cast<int>(i) * 4 + k;
    }
    for (int k = 0; k < 4; ++k) {
      *(it++) = static_cast<int>(i + 1) * 4 + k;
    }
    for (int k = 0; k < static_cast<int>(state_multiplier); ++k) {
      *(it++) = static_cast<int>(num_states) * 4 + static_cast<int>(state_multiplier) * i + k;
    }

    mini_opt::Residual<4, Eigen::Dynamic> integration_equality(
        indices,
        [x0, params](const Eigen::Matrix<double, Eigen::Dynamic, 1>& vars,
                     Eigen::Matrix<double, 4, Eigen::Dynamic>* const J_out) -> Eigen::Vector4d {
          F_ASSERT_EQ(8 + state_multiplier, vars.rows());
          const auto x_k = vars.head<4>();
          const auto x_kpl = vars.middleRows<4>(4);
          const auto u_k = vars.tail(state_multiplier);

          // [x(0) .... x(i)]
          // [u(0), u(1), u(2), u(3), u(4), u(5), u(6), u(7), u(8), u(9)]

          // We need to integrate the state forward and compute the jacobian wrt the control
          // state for each moment in time.
          std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x(state_multiplier);
          std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u(state_multiplier);

          Eigen::Vector4d x = x_k;
          for (std::size_t i = 0; i < state_multiplier; ++i) {
            // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
            // input `u`.
            std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
                x, u_k[i], dt,
                [&params](const auto& x_updated, const double u, auto&& x_dot_D_x,
                          auto&& x_dot_D_u) {
                  Eigen::Matrix<double, 4, 1> x_dot;
                  gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
                                                std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                                std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
                  return x_dot;
                });
            x[1] = mod_pi(x[1]);
          }

          // Go back and compute jacobians:
          if (J_out) {
            F_ASSERT_EQ(8 + state_multiplier, J_out->cols());

            Eigen::Matrix<double, 4, 4> x_kpl_D_x_k = Eigen::Matrix<double, 4, 4>::Identity();
            for (int i = static_cast<int>(state_multiplier) - 1; i >= 0; --i) {
              // Chain rule: final_state_D_state[i+1] * state[i+1]_D_u[i]
              J_out->middleCols<1>(8 + i).noalias() = x_kpl_D_x_k * x_new_D_u[i];
              // Chain rule backwards in time to the previous step:
              x_kpl_D_x_k = (x_kpl_D_x_k * x_new_D_x[i]).eval();
            }

            J_out->leftCols<4>() = x_kpl_D_x_k;
            J_out->middleCols<4>(4) = -Eigen::Matrix<double, 4, 4>::Identity();
          }
          Eigen::Vector4d err = x - x_kpl;
          err[1] = mod_pi(err[1]);
          return err;
        });

    problem.equality_constraints.emplace_back(
        new mini_opt::Residual<4, Eigen::Dynamic>(integration_equality));
  }

  // equality constraints on beginning and end:
  problem.equality_constraints.emplace_back(new mini_opt::Residual<4, 4>(
      {0, 1, 2, 3},
      [x0](const Eigen::Vector4d& vars, Eigen::Matrix<double, 4, 4>* J_out) -> Eigen::Vector4d {
        if (J_out) {
          J_out->setIdentity();
        }
        Eigen::Vector4d err = vars - x0;
        err[1] = mod_pi(err[1]);
        return err;
      }));

  const int last_state_index = static_cast<int>(num_states - 1) * 4;
  problem.equality_constraints.emplace_back(new mini_opt::Residual<3, 4>(
      {last_state_index, last_state_index + 1, last_state_index + 2, last_state_index + 3},
      [](const Eigen::Vector4d& vars, Eigen::Matrix<double, 3, 4>* J_out) -> Eigen::Vector3d {
        if (J_out) {
          J_out->setZero();
          J_out->rightCols<3>().setIdentity();
        }
        return {mod_pi(vars[1] - M_PI / 2), vars[2], vars[3]};
      }));

  for (std::size_t i = 0; i + 1 < window_len; ++i) {
    std::array<int, 2> indices = {static_cast<int>(num_states) * 4 + static_cast<int>(i),
                                  static_cast<int>(num_states) * 4 + static_cast<int>(i + 1)};
    auto function = [](const Eigen::Vector2d& u,
                       Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      constexpr double weight = 0.1;
      if (J_out) {
        J_out->operator()(0, 0) = weight;
        J_out->operator()(0, 1) = -weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
    };
    problem.costs.emplace_back(new mini_opt::Residual<1, 2>(indices, function));
  }

  for (std::size_t i = num_states - 1; i < num_states; ++i) {
    problem.costs.emplace_back(new mini_opt::Residual<1, 1>(
        {static_cast<int>(i * 4)},  // position:
        [](const Eigen::Matrix<double, 1, 1>& vars,
           Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
          constexpr double weight = 150.0;
          if (J_out) {
            J_out->operator[](0) = weight;
          }
          return Eigen::Matrix<double, 1, 1>{vars[0] * weight};
        }));
  }

  for (std::size_t i = 0; i + 1 < num_states; ++i) {
    problem.costs.emplace_back(new mini_opt::Residual<1, 1>(
        {static_cast<int>(i * 4) + 1},  // angle
        [i](const Eigen::Matrix<double, 1, 1>& vars,
            Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
          const double weight = (i + 0.5) / static_cast<double>(num_states);
          constexpr double scale = 200.0;
          const double smooth_weight =
              scale * (3 * std::pow(weight, 2.0) - 2.0 * std::pow(weight, 3));

          const double delta_signed = mod_pi(vars[0] - M_PI / 2);
          if (J_out) {
            J_out->operator[](0) = smooth_weight;
          }
          return Eigen::Matrix<double, 1, 1>{delta_signed * smooth_weight};
        }));
  }

  mini_opt::ConstrainedNonlinearLeastSquares nls(
      &problem, [](Eigen::VectorXd& x, const mini_opt::ConstVectorBlock& dx, const double alpha) {
        x += dx * alpha;
        // Modulo all the angle states:
        for (std::size_t i = 0; i < num_states; ++i) {
          x[i * 4 + 1] = mod_pi(x[i * 4 + 1]);
        }
        for (std::size_t i = 0; i < window_len; ++i) {
          x[num_states * 4 + i] = std::clamp(x[num_states * 4 + i], -600.0, 600.0);
        }
      });

  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = 30;
  p.relative_exit_tol = 1.0e-7;
  p.max_qp_iterations = 1;
  p.max_line_search_iterations = 5;
  p.lambda_initial = 0.0;

  Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(problem.dimension);
  // initial_guess.head<4>() = x0;

  // const double initial_angle = x0[1];
  // const double final_angle = M_PI / 2;
  // for (std::size_t i = 0; i < num_states; ++i) {
  //   initial_guess[i * 4 + 1] = (static_cast<double>(i) + 0.5) / static_cast<double>(num_states) *
  //                                  (final_angle - initial_angle) +
  //                              initial_angle;
  // }

  // for (std::size_t i = 0; i < window_len; ++i) {
  //   initial_guess[num_states * 4 + i] = (i + 0.5) / window_len * 1.0;
  // }

  const mini_opt::NLSSolverOutputs outputs = nls.Solve(p, initial_guess);

  std::cout << fmt::format("Termination state: {}\n{}\n", fmt::streamed(outputs.termination_state),
                           outputs.ToString(true));

  std::cout << nls.variables()
                   .tail(window_len)
                   .transpose()
                   .format(
                       Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]"))
            << std::endl;
}

// ---------------------------------------

TEST(OptimizationTest, TestCartPoleMultipleShootingClosedLoop) {
  // Length of the planning window, and control rate:
  constexpr std::size_t window_len = 50;
  constexpr double dt = 0.01;

  // Parameters of the pendulum system
  constexpr SingleCartPoleParams params{1.0, 0.1, 0.25, 9.81};

  // The initial state of the system:
  const Eigen::Vector4d x0 = (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.0).finished();

  // Total # of parameters:
  constexpr std::size_t state_multiplier = 5;
  static_assert(window_len % state_multiplier == 0);

  // Number of states, plus one more to bound on both sides:
  constexpr std::size_t num_states = (window_len / state_multiplier) + 1;

  mini_opt::Problem problem{};
  problem.dimension = static_cast<int>(num_states * 4 + window_len);

  Eigen::VectorXd prev_solution = Eigen::VectorXd::Zero(problem.dimension);
  Eigen::Vector4d x_current = x0;

  std::vector<Eigen::Vector4d> states{};
  states.push_back(x_current);
  std::vector<double> controls{};
  for (std::size_t t = 0; t < 1; ++t) {
    problem.clear();

    for (std::size_t i = 0; i + 1 < num_states; ++i) {
      std::vector<int> indices;
      auto it = std::back_inserter(indices);
      for (int k = 0; k < 4; ++k) {
        *(it++) = static_cast<int>(i) * 4 + k;
      }
      for (int k = 0; k < 4; ++k) {
        *(it++) = static_cast<int>(i + 1) * 4 + k;
      }
      for (int k = 0; k < static_cast<int>(state_multiplier); ++k) {
        *(it++) = static_cast<int>(num_states) * 4 + static_cast<int>(state_multiplier) * i + k;
      }

      mini_opt::Residual<4, Eigen::Dynamic> integration_equality(
          indices,
          [params](const Eigen::Matrix<double, Eigen::Dynamic, 1>& vars,
                   Eigen::Matrix<double, 4, Eigen::Dynamic>* const J_out) -> Eigen::Vector4d {
            F_ASSERT_EQ(8 + state_multiplier, vars.rows());
            const auto x_k = vars.head<4>();
            const auto x_kpl = vars.middleRows<4>(4);
            const auto u_k = vars.tail(state_multiplier);

            // [x(0) .... x(i)]
            // [u(0), u(1), u(2), u(3), u(4), u(5), u(6), u(7), u(8), u(9)]

            // We need to integrate the state forward and compute the jacobian wrt the control
            // state for each moment in time.
            std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x(state_multiplier);
            std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u(state_multiplier);

            Eigen::Vector4d x = x_k;
            for (std::size_t i = 0; i < state_multiplier; ++i) {
              // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
              // input `u`.
              const auto x_in = x;
              std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
                  x, u_k[i], dt,
                  [&params](const auto& x_updated, const double u, auto&& x_dot_D_x,
                            auto&& x_dot_D_u) {
                    Eigen::Matrix<double, 4, 1> x_dot;
                    gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
                                                  std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                                  std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
                    return x_dot;
                  });
              F_ASSERT(!x.hasNaN(), "x = [{}], x_prev = [{}], u[{}] = {}",
                       fmt::streamed(x.transpose()), fmt::streamed(x_in.transpose()), i, u_k[i]);
              x[1] = mod_pi(x[1]);
            }

            // Go back and compute jacobians:
            if (J_out) {
              F_ASSERT_EQ(8 + state_multiplier, J_out->cols());

              Eigen::Matrix<double, 4, 4> x_kpl_D_x_k = Eigen::Matrix<double, 4, 4>::Identity();
              for (int i = static_cast<int>(state_multiplier) - 1; i >= 0; --i) {
                // Chain rule: final_state_D_state[i+1] * state[i+1]_D_u[i]
                J_out->middleCols<1>(8 + i).noalias() = x_kpl_D_x_k * x_new_D_u[i];
                // Chain rule backwards in time to the previous step:
                x_kpl_D_x_k = (x_kpl_D_x_k * x_new_D_x[i]).eval();
              }

              J_out->leftCols<4>() = x_kpl_D_x_k;
              J_out->middleCols<4>(4) = -Eigen::Matrix<double, 4, 4>::Identity();
            }
            F_ASSERT(!x_kpl.hasNaN(), "x = {}, x_kpl = {}", fmt::streamed(x), fmt::streamed(x_kpl));
            Eigen::Vector4d err = x - x_kpl;
            err[1] = mod_pi(err[1]);
            return err;
          });

      problem.equality_constraints.emplace_back(
          new mini_opt::Residual<4, Eigen::Dynamic>(integration_equality));
    }

    // equality constraints on beginning and end:
    problem.equality_constraints.emplace_back(new mini_opt::Residual<4, 4>(
        {0, 1, 2, 3},
        [x_current](const Eigen::Vector4d& vars,
                    Eigen::Matrix<double, 4, 4>* J_out) -> Eigen::Vector4d {
          if (J_out) {
            J_out->setIdentity();
          }
          Eigen::Vector4d delta = vars - x_current;
          delta[1] = mod_pi(delta[1]);
          return delta;
        }));

    const int last_state_index = static_cast<int>(num_states - 1) * 4;
    problem.equality_constraints.emplace_back(new mini_opt::Residual<3, 4>(
        {last_state_index, last_state_index + 1, last_state_index + 2, last_state_index + 3},
        [](const Eigen::Vector4d& vars, Eigen::Matrix<double, 3, 4>* J_out) -> Eigen::Vector3d {
          if (J_out) {
            J_out->setZero();
            J_out->rightCols<3>().setIdentity();
          }
          return {mod_pi(vars[1] - M_PI / 2), vars[2], vars[3]};
        }));

    for (std::size_t i = 0; i + 1 < window_len; ++i) {
      std::array<int, 2> indices = {static_cast<int>(num_states) * 4 + static_cast<int>(i),
                                    static_cast<int>(num_states) * 4 + static_cast<int>(i + 1)};
      auto function = [](const Eigen::Vector2d& u,
                         Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
        constexpr double weight = 0.1;
        if (J_out) {
          J_out->operator()(0, 0) = weight;
          J_out->operator()(0, 1) = -weight;
        }
        return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
      };
      problem.costs.emplace_back(new mini_opt::Residual<1, 2>(indices, function));
    }

    if (!controls.empty()) {
      std::array<int, 1> indices = {static_cast<int>(num_states) * 4};
      const double u_prev = controls.back();
      auto function = [u_prev](
                          const Eigen::Matrix<double, 1, 1>& u,
                          Eigen::Matrix<double, 1, 1>* const J_out) -> Eigen::Matrix<double, 1, 1> {
        constexpr double weight = 0.1;
        if (J_out) {
          J_out->operator()(0, 0) = weight;
        }
        return Eigen::Matrix<double, 1, 1>{(u[0] - u_prev) * weight};
      };
      problem.costs.emplace_back(new mini_opt::Residual<1, 1>(indices, function));
    }

    for (std::size_t i = num_states - 1; i < num_states; ++i) {
      problem.costs.emplace_back(new mini_opt::Residual<1, 1>(
          {static_cast<int>(i * 4)},  // position:
          [](const Eigen::Matrix<double, 1, 1>& vars,
             Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
            constexpr double weight = 150.0;
            if (J_out) {
              J_out->operator[](0) = weight;
            }
            return Eigen::Matrix<double, 1, 1>{vars[0] * weight};
          }));
    }

    // const double angle_delta = mod_pi(M_PI / 2.0 - x_current[1]);

    for (std::size_t i = 1; i + 1 < num_states; ++i) {
      // const double target_angle =
      //     x_current[1] + (static_cast<double>(i) / static_cast<double>(num_states) *
      //     angle_delta);

      problem.costs.emplace_back(new mini_opt::Residual<1, 1>(
          {static_cast<int>(i * 4) + 1},  // angle
          [t, i](const Eigen::Matrix<double, 1, 1>& vars,
                 Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
            const double weight = (i + 0.5) / static_cast<double>(num_states);
            constexpr double scale = 200.0;
            // const double smooth_weight =
            //     scale * (3 * std::pow(weight, 2.0) - 2.0 * std::pow(weight, 3));

            const double smoother_weight =
                (t < 10000) ? scale * (6 * std::pow(weight, 5) - 15 * std::pow(weight, 4) +
                                       10 * std::pow(weight, 3))
                            : (scale * 20);

            const double delta_signed = mod_pi(vars[0] - M_PI / 2);
            if (J_out) {
              J_out->operator[](0) = smoother_weight;
            }
            return Eigen::Matrix<double, 1, 1>{delta_signed * smoother_weight};
          }));
    }

    mini_opt::ConstrainedNonlinearLeastSquares nls(
        &problem, [](Eigen::VectorXd& x, const mini_opt::ConstVectorBlock& dx, const double alpha) {
          x += dx * alpha;
          // Modulo all the angle states:
          for (std::size_t i = 0; i < num_states; ++i) {
            x[i * 4 + 1] = mod_pi(x[i * 4 + 1]);
          }
          for (std::size_t i = 0; i < window_len; ++i) {
            x[num_states * 4 + i] = std::clamp(x[num_states * 4 + i], -600.0, 600.0);
          }
        });

    mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
    p.max_iterations = 30;
    p.relative_exit_tol = 1.0e-7;
    p.max_qp_iterations = 1;
    p.max_line_search_iterations = 5;
    p.lambda_initial = 0.0;

    Eigen::VectorXd guess = Eigen::VectorXd::Zero(problem.dimension);
    guess.head<4>() = x_current;

    for (std::size_t i = 2; i < num_states; ++i) {
      guess.segment<4>((i - 1) * 4) = prev_solution.segment<4>(i * 4);
    }
    for (std::size_t i = 1; i < window_len; ++i) {
      guess[num_states * 4 + (i - 1)] = prev_solution[num_states * 4 + i];
    }
    guess[guess.rows() - 1] = guess[guess.rows() - 2];

    const auto start = std::chrono::steady_clock::now();
    const mini_opt::NLSSolverOutputs outputs = nls.Solve(p, guess);

    std::cout << fmt::format("Termination state: {} @ {}\n{}\n",
                             fmt::streamed(outputs.termination_state), t, outputs.ToString(true));

    const auto end = std::chrono::steady_clock::now();

    fmt::print("duration (iter = {}): {}\n", t,
               std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1.0e6);

    // if (outputs.termination_state == mini_opt::NLSTerminationState::MAX_ITERATIONS ||
    //     (t >= 236 && t <= 278)) {
    //   std::cout << logger.GetString();
    // }

    // if (true) {
    //   fmt::print("x_initial = [{}]\n", fmt::join(x_current, ", "));
    //   fmt::print("x_final = [{}]\n", fmt::join(nls.variables().head<4>(), ", "));
    //   fmt::print("u = [[{}]]\n", fmt::join(nls.variables().tail(window_len), ", "));
    // }

    // fmt::print("u = [[{}]]\n", fmt::join(nls.variables().tail(window_len), ", "));

    prev_solution = nls.variables();

    // const auto blk = prev_solution.tail(window_len);
    // fmt::print("u({}) = [[{}]]\n", t, fmt::join(blk, ", "));

    // Integrate the system forward:
    const double u = prev_solution[num_states * 4];

    const auto updated = runge_kutta_4th_order<4>(
        x_current, u, dt,
        [&params](const auto& x_updated, const double u, auto&& x_dot_D_x, auto&& x_dot_D_u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
                                        std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                        std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
          return x_dot;
        });
    x_current = std::get<0>(updated);
    x_current[1] = mod_pi(x_current[1]);
    states.push_back(x_current);
    controls.push_back(u);
  }

  // const double initial_angle = x0[1];
  // const double final_angle = M_PI / 2;
  // for (std::size_t i = 0; i < num_states; ++i) {
  //   initial_guess[i * 4 + 1] = (static_cast<double>(i) + 0.5) / static_cast<double>(num_states) *
  //                                  (final_angle - initial_angle) +
  //                              initial_angle;
  // }

  // for (std::size_t i = 0; i < window_len; ++i) {
  //   initial_guess[num_states * 4 + i] = (i + 0.5) / window_len * 1.0;
  // }

  fmt::print("controls: [[{}]]\n", fmt::join(controls, ", "));
  fmt::print("---\n");
  fmt::print("[\n");
  for (const auto& state : states) {
    fmt::print("[{}, {}, {}, {}],\n", state[0], state[1], state[2], state[3]);
  }
  fmt::print("]\n");

  // std::cout << nls.variables()
  //                  .tail(window_len)
  //                  .transpose()
  //                  .format(
  //                      Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]"))
  //           << std::endl;
}

// --------------------------------------------

TEST(OptimizationTest, TestCartPoleMultipleShootingClosedLoop2) {
  constexpr std::size_t num_steps = 200;

  OptimizationParams optimization_params{};
  optimization_params.control_dt = 0.01;
  optimization_params.window_length = 50;
  optimization_params.state_spacing = 5;

  // Parameters of the pendulum system
  constexpr SingleCartPoleParams dynamics_params{1.0, 0.1, 0.25, 9.81};

  // The initial state of the system: -M_PI / 2
  constexpr SingleCartPoleState x0{0.0, 0.0, 0.0, 0.0};

  std::vector<SingleCartPoleState> states{};
  states.reserve(num_steps);
  states.push_back(x0);

  std::vector<double> controls{};
  controls.reserve(num_steps);

  // Simulator will track state and integrate forward.
  Simulator sim{dynamics_params};
  sim.SetState(x0);

  Optimization optimization{optimization_params};

  for (std::size_t t = 0; t < 200; ++t) {
    // Step the optimization and compute a control output:
    const OptimizationOutputs outputs = optimization.Step(sim.GetState(), dynamics_params);

    sim.Step(optimization_params.control_dt, outputs.u.front());

    states.push_back(sim.GetState());
    controls.push_back(outputs.u.front());
  }

  fmt::print("controls: [[{}]]\n", fmt::join(controls, ", "));
  fmt::print("---\n");
  fmt::print("[\n");
  for (const auto& state : states) {
    fmt::print("[{}, {}, {}, {}],\n", state.b_x, state.th_1, state.b_x_dot, state.th_1_dot);
  }
  fmt::print("]\n");
}

}  // namespace pendulum

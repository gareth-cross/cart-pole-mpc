// Copyright 2024 Gareth Cross.
#include "optimization.hpp"

#include "integration.hpp"
#include "key.hpp"
#include "mini_opt/structs.hpp"
#include "single_pendulum_dynamics.hpp"
#include "structs.hpp"

namespace pendulum {

Optimization::Optimization(const OptimizationParams& params) : params_(params) {
  F_ASSERT_GT(params.control_dt, 0);
  F_ASSERT_GE(params.window_length, 1);
  F_ASSERT_EQ(0, params.window_length % params.state_spacing,
              "state_spacing ({}) must divide into window_Length ({}) cleanly",
              params.state_spacing, params.window_length);
  F_ASSERT_GE(params.max_iterations, 1);
  F_ASSERT_GE(params.u_penalty, 0.0);
  F_ASSERT_GE(params.u_derivative_penalty, 0.0);
  F_ASSERT_GE(params.b_x_final_penalty, 0.0);
}

// Map a key to its position in our internal state vector.
// We use the ordering: [x(0), x(1), ... x(N - 1), u(0), u(1), ... u(K - 1)]
// The `x` variables are ordered to match SingleCartPoleState: [b_x, th_1, b_x_dot, th_1_dot]
template <int StateDim>
inline constexpr int MapKey(const KeyType type, const std::size_t index,
                            const std::size_t num_states) noexcept {
  if (type == KeyType::U) {
    // Controls are packed last in the window.
    return static_cast<int>(num_states) * StateDim + static_cast<int>(index);
  } else {
    // The multiple-shooting states are first.
    return static_cast<int>(index) * StateDim + static_cast<int>(type);
  }
}

OptimizationOutputs Optimization::Step(const SingleCartPoleState& current_state,
                                       const SingleCartPoleParams& dynamics_params) {
  // TODO: Don't build all of the problem every iteration. We can re-use the allocated equality
  // constraints.
  BuildProblem(current_state, dynamics_params);

  // Copy the guess from the previous solution:
  Eigen::VectorXd guess = Eigen::VectorXd::Zero(problem_.dimension);

  const auto num_states = params_.NumStates();
  if (previous_solution_.rows() > 0) {
    guess = previous_solution_;
    guess.segment<4>(MapKey<4>(KeyType::B_X, 0, num_states)) = current_state.ToVector();

    // Shift the control inputs from the previous iteration forward by one:
    for (std::size_t k = 0; k + 1 < params_.window_length; ++k) {
      guess[MapKey<4>(KeyType::U, k, num_states)] = guess[MapKey<4>(KeyType::U, k + 1, num_states)];
    }
  } else {
    guess.segment<4>(MapKey<4>(KeyType::B_X, 0, num_states)) = current_state.ToVector();

    // Guess a sinusoidal control pattern. This is a heuristic that meaningfully accelerates
    // convergence. We could just guess zero, but that often produces poor results.
    for (std::size_t k = 0; k < params_.window_length; ++k) {
      guess[MapKey<4>(KeyType::U, k, num_states)] =
          params_.u_guess_sinusoid_amplitude *
          std::sin(static_cast<double>(k) / static_cast<double>(params_.window_length) * 2 * M_PI);
    }
  }

  // Fill the remaining states by integrating control guess values:
  FillInitialGuess(guess, dynamics_params, num_states);

  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = static_cast<int>(params_.max_iterations);
  p.relative_exit_tol = 1.0e-7;
  p.max_line_search_iterations = 5;
  p.absolute_first_derivative_tol = params_.absolute_first_derivative_tol;
  p.equality_penalty_initial = params_.equality_penalty_initial;
  p.relative_exit_tol = params_.relative_exit_tol;

  mini_opt::NLSSolverOutputs outputs = solver_->Solve(p, guess);

  // Copy out the solution to use on the next iteration.
  std::vector<double> previous_solution{previous_solution_.begin(), previous_solution_.end()};
  previous_solution_ = solver_->variables();

  // The control outputs occupy the tail of the state vector:
  const auto u_out =
      static_cast<const Eigen::VectorXd&>(previous_solution_).tail(params_.window_length);

  std::vector<SingleCartPoleState> predicted_states =
      ComputePredictedStates(u_out, dynamics_params, current_state);

#if 0
  const SingleCartPoleState terminal_state = predicted_states.back();
  if (std::abs(terminal_state.b_x_dot) > 2.0 || std::abs(terminal_state.th_1_dot) > 5.0) {
    fmt::print("Will reset the optimization: {}, {}\n", terminal_state.b_x_dot,
               terminal_state.th_1_dot);
    previous_solution_.resize(0);
  }
#endif

  return OptimizationOutputs{current_state, std::move(previous_solution), std::move(outputs),
                             std::vector<double>{u_out.begin(), u_out.end()},
                             std::move(predicted_states)};
}

auto CreateDynamicalConstraint(const SingleCartPoleParams params, const std::size_t state_spacing,
                               const double dt) {
  return [params, state_spacing, dt](
             const Eigen::Matrix<double, Eigen::Dynamic, 1>& vars,
             Eigen::Matrix<double, 4, Eigen::Dynamic>* const J_out) -> Eigen::Vector4d {
    const auto x_k = vars.head<4>();
    const auto x_kpl = vars.middleRows<4>(4);
    const auto u_k = vars.tail(state_spacing);

    // We need to integrate the state forward and compute the jacobian wrt the control
    // state for each moment in time.
    std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x;
    std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u;

    Eigen::Vector4d x = x_k;
    if (J_out) {
      x_new_D_x.resize(state_spacing);
      x_new_D_u.resize(state_spacing);
      for (std::size_t i = 0; i < state_spacing; ++i) {
        // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
        // input `u`.
        std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
            x, u_k[i], dt,
            [&params](const auto& x_updated, const double u, auto&& x_dot_D_x, auto&& x_dot_D_u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
              gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot,
                                            std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                            std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
              return x_dot;
            });
      }
    } else {
      for (std::size_t i = 0; i < state_spacing; ++i) {
        x = runge_kutta_4th_order_no_jacobians(
            x, u_k[i], dt, [&params](const auto& x_updated, const double u) {
              Eigen::Matrix<double, 4, 1> x_dot;
              const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
              gen::single_pendulum_dynamics(params, x_updated, u, zero, zero, x_dot, nullptr,
                                            nullptr);
              return x_dot;
            });
      }
    }
    x[1] = mod_pi(x[1]);

    // Go back and compute jacobians:
    if (J_out) {
      F_ASSERT_EQ(8 + state_spacing, static_cast<std::size_t>(J_out->cols()));

      Eigen::Matrix<double, 4, 4> x_kpl_D_x_k = Eigen::Matrix<double, 4, 4>::Identity();
      for (int i = static_cast<int>(state_spacing) - 1; i >= 0; --i) {
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
  };
}

void Optimization::BuildProblem(const SingleCartPoleState& current_state,
                                const SingleCartPoleParams& dynamics_params) {
  static constexpr int state_dim = 4;
  const std::size_t num_states = params_.NumStates();

  problem_.clear();
  problem_.dimension =
      static_cast<int>(num_states) * state_dim + static_cast<int>(params_.window_length);

  // Insert the integration equality constraints between adjacent states:
  for (std::size_t s = 0; s + 1 < num_states; ++s) {
    std::vector<int> indices;
    indices.reserve(20);
    for (const std::size_t state : {s, s + 1}) {
      indices.push_back(MapKey<state_dim>(KeyType::B_X, state, num_states));
      indices.push_back(MapKey<state_dim>(KeyType::THETA_1, state, num_states));
      indices.push_back(MapKey<state_dim>(KeyType::B_X_DOT, state, num_states));
      indices.push_back(MapKey<state_dim>(KeyType::THETA_1_DOT, state, num_states));
    }
    for (std::size_t k = 0; k < params_.state_spacing; ++k) {
      indices.push_back(MapKey<state_dim>(KeyType::U, params_.state_spacing * s + k, num_states));
    }

    auto cost =
        CreateDynamicalConstraint(dynamics_params, params_.state_spacing, params_.control_dt);
    problem_.equality_constraints.emplace_back(
        new mini_opt::Residual<4, Eigen::Dynamic>(std::move(indices), cost));
  }

  // Equality constraint on the initial state:
  const auto x_current = current_state.ToVector();
  problem_.equality_constraints.emplace_back(new mini_opt::Residual<4, 4>(
      {MapKey<state_dim>(KeyType::B_X, 0, num_states),
       MapKey<state_dim>(KeyType::THETA_1, 0, num_states),
       MapKey<state_dim>(KeyType::B_X_DOT, 0, num_states),
       MapKey<state_dim>(KeyType::THETA_1_DOT, 0, num_states)},
      [x_current](const Eigen::Vector4d& vars,
                  Eigen::Matrix<double, 4, 4>* J_out) -> Eigen::Vector4d {
        if (J_out) {
          J_out->setIdentity();
        }
        Eigen::Vector4d delta = vars - x_current;
        delta[1] = mod_pi(delta[1]);
        return delta;
      }));

  if (params_.terminal_angle_constraint_enabled) {
    problem_.equality_constraints.emplace_back(new mini_opt::Residual<1, 1>(
        {MapKey<state_dim>(KeyType::THETA_1, num_states - 1, num_states)},
        [](const Eigen::Matrix<double, 1, 1>& vars,
           Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
          if (J_out) {
            J_out->setIdentity();
          }
          return Eigen::Matrix<double, 1, 1>{mod_pi(vars[0] - M_PI / 2)};
        }));
  }

  // Equality constraint on the final state (velocities are zero).
  problem_.equality_constraints.emplace_back(new mini_opt::Residual<2, 2>(
      {MapKey<state_dim>(KeyType::B_X_DOT, num_states - 1, num_states),
       MapKey<state_dim>(KeyType::THETA_1_DOT, num_states - 1, num_states)},
      [](const Eigen::Vector2d& vars, Eigen::Matrix<double, 2, 2>* J_out) -> Eigen::Vector2d {
        if (J_out) {
          J_out->setIdentity();
        }
        return {vars[0], vars[1]};
      }));

  // Quadratic penalty on the derivative of control inputs:
  for (std::size_t k = 0; k + 1 < params_.window_length; ++k) {
    const double weight = params_.u_derivative_penalty;
    auto cost = [weight](const Eigen::Vector2d& u,
                         Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      if (J_out) {
        J_out->operator()(0, 0) = weight;
        J_out->operator()(0, 1) = -weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
    };
    problem_.costs.emplace_back(
        new mini_opt::Residual<1, 2>({MapKey<state_dim>(KeyType::U, k, num_states),
                                      MapKey<state_dim>(KeyType::U, k + 1, num_states)},
                                     cost));
  }

  // And between our current control input and the one from the last iteration:
  const double u_prev = previous_solution_.rows() > 0
                            ? previous_solution_[MapKey<4>(KeyType::U, 0, num_states)]
                            : 0.0;
  const double u_initial_weight = params_.u_derivative_penalty;
  auto cost_initial_control =
      [u_prev, u_initial_weight](
          const Eigen::Matrix<double, 1, 1>& u,
          Eigen::Matrix<double, 1, 1>* const J_out) -> Eigen::Matrix<double, 1, 1> {
    if (J_out) {
      J_out->operator()(0, 0) = u_initial_weight;
    }
    return Eigen::Matrix<double, 1, 1>{(u[0] - u_prev) * u_initial_weight};
  };
  problem_.costs.emplace_back(new mini_opt::Residual<1, 1>(
      {MapKey<state_dim>(KeyType::U, 0, num_states)}, cost_initial_control));

  // Quadratic penalty on the final position, drive the cart back to the center:
  const double b_x_final_weight = params_.b_x_final_penalty;
  problem_.costs.emplace_back(new mini_opt::Residual<1, 1>(
      {MapKey<state_dim>(KeyType::B_X, num_states - 1, num_states)},
      [b_x_final_weight](const Eigen::Matrix<double, 1, 1>& vars,
                         Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
        if (J_out) {
          J_out->operator[](0) = b_x_final_weight;
        }
        return Eigen::Matrix<double, 1, 1>{vars[0] * b_x_final_weight};
      }));

  for (std::size_t k = 0; k < params_.window_length; ++k) {
    const double u_penalty = params_.u_penalty;
    auto cost = [u_penalty](
                    const Eigen::Matrix<double, 1, 1>& u,
                    Eigen::Matrix<double, 1, 1>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      if (J_out) {
        J_out->operator()(0, 0) = u_penalty;
      }
      return Eigen::Matrix<double, 1, 1>{u[0] * u_penalty};
    };
    problem_.costs.emplace_back(
        new mini_opt::Residual<1, 1>({MapKey<state_dim>(KeyType::U, k, num_states)}, cost));
  }

  if (solver_) {
    // Initialize the solver once and re-use it.
    return;
  }

  const std::size_t window_length = params_.window_length;
  const auto retraction = [num_states, window_length](Eigen::VectorXd& x,
                                                      const mini_opt::ConstVectorBlock dx,
                                                      const double alpha) {
    x.noalias() += dx * alpha;
    for (std::size_t s = 0; s < num_states; ++s) {
      // Modulo all the angle states:
      const int angle_index = MapKey<state_dim>(KeyType::THETA_1, s, num_states);
      x[angle_index] = mod_pi(x[angle_index]);

      // To keep ths system from blowing up too much numerically we clamp b_x(t)...
      const int pos_index = MapKey<state_dim>(KeyType::B_X, s, num_states);
      x[pos_index] = std::clamp(x[pos_index], -5.0, 5.0);
    }

    // ... And the control inputs u(t).
    // TODO: Make parameters for these limits.
    for (std::size_t k = 0; k < window_length; ++k) {
      const int index = MapKey<state_dim>(KeyType::U, k, num_states);
      x[index] = std::clamp(x[index], -300.0, 300.0);
    }
  };
  solver_ = std::make_unique<mini_opt::ConstrainedNonlinearLeastSquares>(&problem_, retraction);
}

void Optimization::FillInitialGuess(Eigen::VectorXd& guess,
                                    const SingleCartPoleParams& dynamics_params,
                                    const std::size_t num_states) const {
  Eigen::Vector4d x = guess.segment<4>(MapKey<4>(KeyType::B_X, 0, num_states));
  for (std::size_t s = 1; s < num_states; ++s) {
    // Integrate up all the samples between x(s - 1) and x(s):
    for (std::size_t k = 0; k < params_.state_spacing; ++k) {
      x = runge_kutta_4th_order_no_jacobians<4>(
          x, guess[MapKey<4>(KeyType::U, (s - 1) * params_.state_spacing + k, num_states)],
          params_.control_dt, [&dynamics_params](const auto& x_updated, const double u) {
            Eigen::Matrix<double, 4, 1> x_dot;
            const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
            gen::single_pendulum_dynamics(dynamics_params, x_updated, u, zero, zero, x_dot, nullptr,
                                          nullptr);
            return x_dot;
          });
      x[1] = mod_pi(x[1]);
    }
    guess.segment<4>(MapKey<4>(KeyType::B_X, s, num_states)).noalias() = x;
  }
}

std::vector<SingleCartPoleState> Optimization::ComputePredictedStates(
    const mini_opt::ConstVectorBlock u_out, const SingleCartPoleParams& dynamics_params,
    const SingleCartPoleState& x_current) const {
  std::vector<SingleCartPoleState> predicted_states;
  predicted_states.reserve(u_out.rows());

  Eigen::Vector4d x = x_current.ToVector();
  for (std::size_t k = 0; k < params_.window_length; ++k) {
    x = runge_kutta_4th_order_no_jacobians<4>(
        x, u_out[k], params_.control_dt, [&dynamics_params](const auto& x_updated, const double u) {
          Eigen::Matrix<double, 4, 1> x_dot;
          const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
          gen::single_pendulum_dynamics(dynamics_params, x_updated, u, zero, zero, x_dot, nullptr,
                                        nullptr);
          return x_dot;
        });
    x[1] = mod_pi(x[1]);
    predicted_states.emplace_back(x);
  }
  return predicted_states;
}

}  // namespace pendulum

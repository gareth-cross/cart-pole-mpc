// Copyright 2024 Gareth Cross.
#include "optimization.hpp"

#include <mini_opt/logging.hpp>

#include "integration.hpp"
#include "key.hpp"
#include "single_pendulum_dynamics.hpp"

namespace pendulum {

Optimization::Optimization(const OptimizationParams& params) : params_(params) {
  F_ASSERT_GT(params.control_dt, 0);
  F_ASSERT_GE(params.window_length, 1);
  F_ASSERT_EQ(0, params.window_length % params.state_spacing,
              "state_spacing ({}) must divide into window_Length ({}) cleanly",
              params.state_spacing, params.window_length);
}

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

std::tuple<mini_opt::NLSSolverOutputs, double> Optimization::Step(
    const SingleCartPoleState& current_state, const SingleCartPoleParams& dynamics_params) {
  // TODO: Don't build the problem every iteration.
  BuildProblem(current_state, dynamics_params);

  // Copy the guess from the previous solution:
  Eigen::VectorXd guess = Eigen::VectorXd::Zero(problem_.dimension);

  const auto num_states = params_.NumStates();
  if (previous_solution_.rows() > 0) {
    const int first_control = MapKey<4>(KeyType::U, 0, num_states);

    guess.head(first_control) = previous_solution_.head(first_control);
    guess[MapKey<4>(KeyType::B_X, 0, num_states)] = current_state.b_x;
    guess[MapKey<4>(KeyType::THETA_1, 0, num_states)] = current_state.th_1;
    guess[MapKey<4>(KeyType::B_X_DOT, 0, num_states)] = current_state.b_x_dot;
    guess[MapKey<4>(KeyType::THETA_1_DOT, 0, num_states)] = current_state.th_1_dot;

    guess.segment(first_control, guess.rows() - first_control - 2) =
        previous_solution_.segment(first_control + 1, guess.rows() - first_control - 1);
  }

  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = 200;
  p.relative_exit_tol = 1.0e-7;
  p.max_qp_iterations = 1;
  p.max_line_search_iterations = 5;
  p.lambda_initial = 0.0;

  mini_opt::Logger logger{};
  solver_->SetLoggingCallback(std::bind(&mini_opt::Logger::NonlinearSolverCallback, &logger,
                                        std::placeholders::_1, std::placeholders::_2));

  const mini_opt::NLSSolverOutputs outputs = solver_->Solve(p, guess);

  fmt::print("Termination state: {}\n", fmt::streamed(outputs.termination_state));

  // Copy out the solution:
  previous_solution_ = solver_->variables();

  return std::make_tuple(outputs, previous_solution_[MapKey<4>(KeyType::U, 0, num_states)]);
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
    std::vector<Eigen::Matrix<double, 4, 4>> x_new_D_x(state_spacing);
    std::vector<Eigen::Matrix<double, 4, 1>> x_new_D_u(state_spacing);

    Eigen::Vector4d x = x_k;
    for (std::size_t i = 0; i < state_spacing; ++i) {
      // Integrate forward to get new `x`, and the jacobians wrt previous x and the control
      // input `u`.
      std::tie(x, x_new_D_x[i], x_new_D_u[i]) = runge_kutta_4th_order<4>(
          x, u_k[i], dt,
          [&params](const auto& x_updated, const double u, auto&& x_dot_D_x, auto&& x_dot_D_u) {
            Eigen::Matrix<double, 4, 1> x_dot;
            gen::single_pendulum_dynamics(params, x_updated, u, x_dot,
                                          std::forward<decltype(x_dot_D_x)>(x_dot_D_x),
                                          std::forward<decltype(x_dot_D_u)>(x_dot_D_u));
            return x_dot;
          });
      F_ASSERT(!x.hasNaN(), "x = [{}], u[{}] = {}", fmt::streamed(x.transpose()), i, u_k[i]);
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
  constexpr int state_dim = 4;

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

  // Equality constraint on the final state (velocities are zero, pole is vertical).
  problem_.equality_constraints.emplace_back(new mini_opt::Residual<3, 3>(
      {MapKey<state_dim>(KeyType::THETA_1, num_states - 1, num_states),
       MapKey<state_dim>(KeyType::B_X_DOT, num_states - 1, num_states),
       MapKey<state_dim>(KeyType::THETA_1_DOT, num_states - 1, num_states)},
      [](const Eigen::Vector3d& vars, Eigen::Matrix<double, 3, 3>* J_out) -> Eigen::Vector3d {
        if (J_out) {
          J_out->setIdentity();
        }
        return {mod_pi(vars[0] - M_PI / 2), vars[1], vars[2]};
      }));

  // Quadratic penalty on the derivative of control inputs:
  for (std::size_t k = 0; k + 1 < params_.window_length; ++k) {
    const std::array<int, 2> indices = {MapKey<state_dim>(KeyType::U, k, num_states),
                                        MapKey<state_dim>(KeyType::U, k + 1, num_states)};
    auto cost = [](const Eigen::Vector2d& u,
                   Eigen::Matrix<double, 1, 2>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      constexpr double weight = 0.1;
      if (J_out) {
        J_out->operator()(0, 0) = weight;
        J_out->operator()(0, 1) = -weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u[1]) * weight};
    };
    problem_.costs.emplace_back(new mini_opt::Residual<1, 2>(indices, cost));
  }

  // And between our current control input and the last one:
  if (previous_solution_.rows() > 0) {
    const double u_prev = previous_solution_[MapKey<4>(KeyType::U, 0, num_states)];
    auto cost = [u_prev](const Eigen::Matrix<double, 1, 1>& u,
                         Eigen::Matrix<double, 1, 1>* const J_out) -> Eigen::Matrix<double, 1, 1> {
      constexpr double weight = 0.1;
      if (J_out) {
        J_out->operator()(0, 0) = weight;
      }
      return Eigen::Matrix<double, 1, 1>{(u[0] - u_prev) * weight};
    };
    problem_.costs.emplace_back(
        new mini_opt::Residual<1, 1>({MapKey<state_dim>(KeyType::U, 0, num_states)}, cost));
  }

  // Quadratic penalty on the final position, drive the cart back to the center:
  problem_.costs.emplace_back(new mini_opt::Residual<1, 1>(
      {MapKey<state_dim>(KeyType::B_X, num_states - 1, num_states)},
      [](const Eigen::Matrix<double, 1, 1>& vars,
         Eigen::Matrix<double, 1, 1>* J_out) -> Eigen::Matrix<double, 1, 1> {
        constexpr double weight = 150.0;
        if (J_out) {
          J_out->operator[](0) = weight;
        }
        return Eigen::Matrix<double, 1, 1>{vars[0] * weight};
      }));

  //  Quadratic cost against the reference trajectory.
  for (std::size_t i = 1; i + 1 < num_states; ++i) {
    problem_.costs.emplace_back(new mini_opt::Residual<1, 1>(
        {MapKey<state_dim>(KeyType::THETA_1, i, num_states)},
        [num_states, i](const Eigen::Matrix<double, 1, 1>& vars,
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

  const std::size_t window_length = params_.window_length;
  const auto retraction = [num_states, window_length](Eigen::VectorXd& x,
                                                      const mini_opt::ConstVectorBlock& dx,
                                                      const double alpha) {
    x.noalias() += dx * alpha;
    // Modulo all the angle states:
    for (std::size_t s = 0; s < num_states; ++s) {
      const int index = MapKey<state_dim>(KeyType::THETA_1, s, num_states);
      x[index] = mod_pi(x[index]);
    }
    // Clamp control inputs:
    for (std::size_t k = 0; k < window_length; ++k) {
      const int index = MapKey<state_dim>(KeyType::U, k, num_states);
      x[index] = std::clamp(x[index], -600.0, 600.0);
    }
  };
  solver_ = std::make_unique<mini_opt::ConstrainedNonlinearLeastSquares>(&problem_, retraction);
}

}  // namespace pendulum

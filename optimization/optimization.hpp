// Copyright 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

#include <mini_opt/nonlinear.hpp>

#include "structs.hpp"

namespace pendulum {

struct OptimizationParams {
  // Step between sequential control inputs in the planning window.
  double control_dt{0.01};
  // Length of the planning horizon in samples.
  std::size_t window_length{50};
  // # of control inputs between sequential states in the optimization.
  // Setting this to `1` would correspond to multiple shooting.
  std::size_t state_spacing{5};

  // Number of states in the window.
  // Add one for the terminal state.
  constexpr std::size_t NumStates() const noexcept { return window_length / state_spacing + 1; }
};

// Implementation of hybrid multiple-shooting MPC.
class Optimization {
 public:
  explicit Optimization(const OptimizationParams& params);

  // Run an iteration of optimization and compute control outputs.
  std::tuple<mini_opt::NLSSolverOutputs, double> Step(const SingleCartPoleState& current_state,
                                                      const SingleCartPoleParams& dynamics_params);

 private:
  void BuildProblem(const SingleCartPoleState& current_state,
                    const SingleCartPoleParams& dynamics_params);

  OptimizationParams params_;
  mini_opt::Problem problem_;
  std::unique_ptr<mini_opt::ConstrainedNonlinearLeastSquares> solver_;
  Eigen::VectorXd previous_solution_;
};

}  // namespace pendulum

// Copyright (c) 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

#include <mini_opt/nonlinear.hpp>

#include "mini_opt/structs.hpp"
#include "structs.hpp"

namespace pendulum {

struct OptimizationParams {
  // Step between sequential control inputs in the planning window.
  double control_dt{0.01};

  // Length of the planning horizon in samples.
  std::size_t window_length{40};

  // # of control inputs between sequential states in the optimization.
  // Setting this to `1` would correspond to multiple shooting.
  // Setting this to `window_length` would roughly correspond to single shooting.
  std::size_t state_spacing{10};

  // Max iterations of optimization.
  std::size_t max_iterations{8};

  // See mini_opt::ConstrainedNonlinearLeastSquares::Params.
  double relative_exit_tol{1.0e-5};

  // See mini_opt::ConstrainedNonlinearLeastSquares::Params.
  double absolute_first_derivative_tol{1.0e-6};

  // See mini_opt::ConstrainedNonlinearLeastSquares::Params.
  double equality_penalty_initial{1.0};

  // Amplitude of the sinusoid that we feed in as our initial guess.
  double u_guess_sinusoid_amplitude{10.0};

  // Parameters on the quadratic weights in the optimization:
  double u_penalty{0.1};
  double u_derivative_penalty{0.1};
  double b_x_final_penalty{150.0};

  // Number of states in the window.
  // Add one for the terminal state.
  constexpr std::size_t NumStates() const noexcept { return window_length / state_spacing + 1; }
};

struct OptimizationOutputs {
  // The initial state.
  SingleCartPoleState initial_state;

  // The initial guess for all variables.
  std::vector<double> previous_solution;

  // Debug statistics from the solver itself.
  mini_opt::NLSSolverOutputs solver_outputs;

  // Control actions over the optimized window.
  std::vector<double> u;

  // Predicted states integrated over the control window.
  std::vector<SingleCartPoleState> predicted_states;
};

// Implementation of hybrid multiple-shooting MPC.
class Optimization {
 public:
  explicit Optimization(const OptimizationParams& params);

  // Run an iteration of optimization and compute control outputs.
  [[nodiscard]] OptimizationOutputs Step(const SingleCartPoleState& current_state,
                                         const SingleCartPoleParams& dynamics_params);

  // Discard previous initial guess, which will reset the problem.
  void Reset() { previous_solution_.resize(0); }

  // Set the previous solution, which will be used as guess on the next iteration.
  void SetPreviousSolution(const std::vector<double>& guess) {
    previous_solution_.resize(guess.size());
    std::copy(guess.begin(), guess.end(), previous_solution_.begin());
  }

 private:
  void BuildProblem(const SingleCartPoleState& current_state,
                    const SingleCartPoleParams& dynamics_params);

  // Fill initial guess vector by integrating the dynamics model.
  void FillInitialGuess(Eigen::VectorXd& guess, const SingleCartPoleParams& dynamics_params,
                        const std::size_t num_states) const;

  // Given the solution vector, compute the full set of states over the window.
  std::vector<SingleCartPoleState> ComputePredictedStates(
      mini_opt::ConstVectorBlock u_out, const SingleCartPoleParams& dynamics_params,
      const SingleCartPoleState& x_current) const;

  OptimizationParams params_;
  mini_opt::Problem problem_;
  std::unique_ptr<mini_opt::ConstrainedNonlinearLeastSquares> solver_;
  Eigen::VectorXd previous_solution_;
};

}  // namespace pendulum

// Copyright (c) 2024 Gareth Cross.
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "optimization.hpp"
#include "simulator.hpp"

namespace pendulum {

// A very basic test that we can optimize and get a solution that produces the desired terminal
// states.
TEST(OptimizationTest, TestCartPoleMultipleShootingClosedLoop) {
  constexpr std::size_t num_steps = 200;

  OptimizationParams optimization_params{};
  optimization_params.control_dt = 0.01;
  optimization_params.window_length = 40;
  optimization_params.state_spacing = 5;
  optimization_params.max_iterations = 10;

  // Parameters of the pendulum system
  constexpr SingleCartPoleParams dynamics_params{1.0, 0.1, 0.25, 9.81, 0.03, 0.1, 0.13, 0.8, 100.0};

  // The initial state of the system: -pi / 2
  constexpr SingleCartPoleState x0{0.0, -M_PI / 2, 0.0, 0.0};

  std::vector<SingleCartPoleState> states{};
  states.reserve(num_steps);
  states.push_back(x0);

  std::vector<double> controls{};
  controls.reserve(num_steps);

  // Simulator will track state and integrate forward.
  Simulator sim{};
  sim.SetState(x0);

  Optimization optimization{optimization_params};
  for (std::size_t t = 0; t < num_steps; ++t) {
    // Step the optimization and compute a control output:
    const OptimizationOutputs outputs = optimization.Step(sim.GetState(), dynamics_params);

    // Were the end-points satisfied?
    ASSERT_NE(mini_opt::NLSTerminationState::QP_INDEFINITE,
              outputs.solver_outputs.termination_state);
    ASSERT_NE(mini_opt::NLSTerminationState::MAX_LAMBDA, outputs.solver_outputs.termination_state);

    const SingleCartPoleState& terminal_state = outputs.predicted_states.back();

    // Takes a few iterations to reach good convergence:
    if (t > 20) {
      ASSERT_NEAR(0.0, terminal_state.b_x_dot, 1.0e-4);
      ASSERT_NEAR(0.0, terminal_state.th_1_dot, 1.0e-4);
      ASSERT_NEAR(M_PI / 2, terminal_state.th_1, 1.0e-4);
    }

    states.push_back(sim.GetState());
    controls.push_back(outputs.u.front());

    sim.Step(dynamics_params, optimization_params.control_dt, outputs.u.front(), {0, 0}, {0, 0});
  }

  const SingleCartPoleState& terminal_state = states.back();
  ASSERT_NEAR(0.0, terminal_state.b_x_dot, 1.0e-4);
  ASSERT_NEAR(0.0, terminal_state.th_1_dot, 1.0e-3);
  ASSERT_NEAR(M_PI / 2, terminal_state.th_1, 1.0e-4);

#if 0
  fmt::print("controls: [[{}]]\n", fmt::join(controls, ", "));
  fmt::print("---\n");
  fmt::print("[\n");
  for (const auto& state : states) {
    fmt::print("[{}, {}, {}, {}],\n", state.b_x, state.th_1, state.b_x_dot, state.th_1_dot);
  }
  fmt::print("]\n");
#endif
}

}  // namespace pendulum

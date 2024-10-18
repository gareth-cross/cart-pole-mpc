// Copyright (c) 2024 Gareth Cross.
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "optimization.hpp"
#include "simulator.hpp"

namespace pendulum {

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
  Simulator sim{dynamics_params};
  sim.SetState(x0);

  Optimization optimization{optimization_params};
  for (std::size_t t = 0; t < num_steps; ++t) {
    // Step the optimization and compute a control output:
    const OptimizationOutputs outputs = optimization.Step(sim.GetState(), dynamics_params);

    sim.Step(optimization_params.control_dt, outputs.u.front(), {0, 0}, {0, 0});

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

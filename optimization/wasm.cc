// Copyright 2024 Gareth Cross.
// WASM module for interacting with the controller.
#include <emscripten/bind.h>

#include <sanitizer/lsan_interface.h>

#include "optimization.hpp"
#include "simulator.hpp"

namespace em = emscripten;
using namespace pendulum;

EMSCRIPTEN_BINDINGS(OptimizationWasm) {
  em::class_<SingleCartPoleState>("SingleCartPoleState")
      .constructor<double, double, double, double>()
      .property("b_x", &SingleCartPoleState::b_x)
      .property("th_1", &SingleCartPoleState::th_1)
      .property("b_x_dot", &SingleCartPoleState::b_x_dot)
      .property("th_1_dot", &SingleCartPoleState::th_1_dot);

  em::class_<SingleCartPoleParams>("SingleCartPoleParams")
      .constructor<double, double, double, double>()
      .property("m_b", &SingleCartPoleParams::m_b)
      .property("m_1", &SingleCartPoleParams::m_1)
      .property("l_1", &SingleCartPoleParams::l_1)
      .property("g", &SingleCartPoleParams::g);

  em::class_<Simulator>("Simulator")
      .constructor<SingleCartPoleParams>()
      .function("step", &Simulator::Step)
      .function("getState", &Simulator::GetState)
      .function("setState", &Simulator::SetState);

  em::class_<OptimizationParams>("OptimizationParams")
      .constructor<>()
      .property("control_dt", &OptimizationParams::control_dt)
      .property("window_length", &OptimizationParams::window_length)
      .property("state_spacing", &OptimizationParams::state_spacing)
      .property("max_iterations", &OptimizationParams::max_iterations)
      .property("relative_exit_tol", &OptimizationParams::relative_exit_tol)
      .property("absolute_first_derivative_tol", &OptimizationParams::absolute_first_derivative_tol)
      .property("u_guess_sinusoid_amplitude", &OptimizationParams::u_guess_sinusoid_amplitude)
      .property("u_penalty", &OptimizationParams::u_penalty)
      .property("u_derivative_penalty", &OptimizationParams::u_derivative_penalty)
      .property("b_x_final_penalty", &OptimizationParams::b_x_final_penalty);

  em::class_<OptimizationOutputs>("OptimizationOutputs")
      .function(
          "getLog", +[](const OptimizationOutputs& self) { return self.solver_outputs.ToString(); })
      .function(
          "windowLength", +[](const OptimizationOutputs& self) { return self.u.size(); })
      .function(
          "getControl",
          +[](const OptimizationOutputs& self, std::size_t index) {
            F_ASSERT_LT(index, self.u.size());
            return self.u[index];
          })
      .function(
          "getPredictedState",
          +[](const OptimizationOutputs& self, std::size_t index) -> const SingleCartPoleState* {
            // This is returned by reference, so it is only valid until OptimizationOutputs
            // is deleted.
            F_ASSERT_LT(index, self.predicted_states.size());
            return &self.predicted_states[index];
          },
          em::return_value_policy::reference());

  em::class_<Optimization>("Optimization")
      .constructor<OptimizationParams>()
      .function("step", &Optimization::Step);

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
  em::function("doLeakCheck", &__lsan_do_recoverable_leak_check);
#endif
#endif
}

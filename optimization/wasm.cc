// Copyright 2024 Gareth Cross.
// WASM module for interacting with the controller.
#include <emscripten/bind.h>

#include <sanitizer/lsan_interface.h>

#include "optimization.hpp"
#include "simulator.hpp"

namespace pendulum {}  // namespace pendulum

namespace em = emscripten;

EMSCRIPTEN_BINDINGS(OptimizationWasm) {
  em::class_<pendulum::SingleCartPoleState>("SingleCartPoleState")
      .constructor<double, double, double, double>()
      .property("b_x", &pendulum::SingleCartPoleState::b_x)
      .property("th_1", &pendulum::SingleCartPoleState::th_1)
      .property("b_x_dot", &pendulum::SingleCartPoleState::b_x_dot)
      .property("th_1_dot", &pendulum::SingleCartPoleState::th_1_dot);

  em::class_<pendulum::SingleCartPoleParams>("SingleCartPoleParams")
      .constructor<double, double, double, double>()
      .property("m_b", &pendulum::SingleCartPoleParams::m_b)
      .property("m_1", &pendulum::SingleCartPoleParams::m_1)
      .property("l_1", &pendulum::SingleCartPoleParams::l_1)
      .property("g", &pendulum::SingleCartPoleParams::g);

  em::class_<pendulum::Simulator>("Simulator")
      .constructor<pendulum::SingleCartPoleParams>()
      .function("step", &pendulum::Simulator::Step)
      .function("getState", &pendulum::Simulator::GetState)
      .function("setState", &pendulum::Simulator::SetState);

  em::class_<pendulum::OptimizationParams>("OptimizationParams")
      .constructor<>()
      .property("control_dt", &pendulum::OptimizationParams::control_dt)
      .property("window_length", &pendulum::OptimizationParams::window_length)
      .property("state_spacing", &pendulum::OptimizationParams::state_spacing);

  em::class_<pendulum::OptimizationOutputs>("OptimizationOutputs")
      .function(
          "getLog",
          +[](const pendulum::OptimizationOutputs& self) { return self.solver_outputs.ToString(); })
      .function(
          "windowLength", +[](const pendulum::OptimizationOutputs& self) { return self.u.size(); })
      .function(
          "getControl",
          +[](const pendulum::OptimizationOutputs& self, std::size_t index) {
            F_ASSERT_LT(index, self.u.size());
            return self.u[index];
          })
      .function(
          "getPredictedState",
          +[](const pendulum::OptimizationOutputs& self,
              std::size_t index) -> const pendulum::SingleCartPoleState* {
            // This is returned by reference, so it is only valid until OptimizationOutputs
            // is deleted.
            F_ASSERT_LT(index, self.predicted_states.size());
            return &self.predicted_states[index];
          },
          em::return_value_policy::reference());

  em::class_<pendulum::Optimization>("Optimization")
      .constructor<pendulum::OptimizationParams>()
      .function("step", &pendulum::Optimization::Step);

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
  em::function("doLeakCheck", &__lsan_do_recoverable_leak_check);
#endif
#endif
}

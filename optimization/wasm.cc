// Copyright 2024 Gareth Cross.
// WASM module for interacting with the controller.
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <sanitizer/lsan_interface.h>
#include <nlohmann/json.hpp>

#include <mini_opt/serialization.hpp>
#include <mini_opt/tracing.hpp>

#include "optimization.hpp"
#include "simulator.hpp"

namespace em = emscripten;

namespace pendulum {
using json = nlohmann::json;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SingleCartPoleState, b_x, th_1, th_1_dot, b_x_dot);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SingleCartPoleParams, m_b, m_1, l_1, g, mu_b, v_mu_b, c_d_1, x_s,
                                   k_s);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector2, x, y);

// Encode to JSON, then decode in JavaScript.
template <typename T>
em::val ObjectFromStruct(const T& s) {
  const std::string code = json(s).dump();
  return em::val::global("JSON").call<em::val>("parse", code);
}

// Encode to JSON, then decode into C++ struct.
template <typename T>
T StructFromObject(em::val obj) {
  const std::string code = em::val::global("JSON").call<std::string>("stringify", obj);
  return json::parse(code).get<T>();
}

}  // namespace pendulum

namespace nlohmann {
template <>
struct adl_serializer<pendulum::OptimizationOutputs> {
  static pendulum::OptimizationOutputs from_json(const json& j) {
    return pendulum::OptimizationOutputs{
        j["initial_state"].get<pendulum::SingleCartPoleState>(),
        j["previous_solution"].get<std::vector<double>>(),
        j["solver_outputs"].get<mini_opt::NLSSolverOutputs>(), j["u"].get<std::vector<double>>(),
        j["predicted_states"].get<std::vector<pendulum::SingleCartPoleState>>()};
  }

  static void to_json(json& j, const pendulum::OptimizationOutputs& outputs) {
    j["initial_state"] = outputs.initial_state;
    j["previous_solution"] = outputs.previous_solution;
    j["solver_outputs"] = outputs.solver_outputs;
    j["u"] = outputs.u;
    j["predicted_states"] = outputs.predicted_states;
  }
};
}  // namespace nlohmann

using namespace pendulum;

EMSCRIPTEN_BINDINGS(OptimizationWasm) {
  em::class_<Simulator>("Simulator")
      .constructor(
          +[](em::val params) { return Simulator(StructFromObject<SingleCartPoleParams>(params)); })
      .function(
          "step",
          +[](Simulator& sim, double dt, double u, em::val f_external) {
            const auto external_forces = StructFromObject<std::vector<Vector2>>(f_external);
            F_ASSERT_EQ(2, external_forces.size());
            return sim.Step(dt, u, external_forces[0], external_forces[1]);
          })
      .function(
          "getState", +[](const Simulator& sim) { return ObjectFromStruct(sim.GetState()); })
      .function(
          "setState", +[](Simulator& sim, em::val state) {
            sim.SetState(StructFromObject<SingleCartPoleState>(state));
          });

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
          +[](const OptimizationOutputs& self, std::size_t index) {
            F_ASSERT_LT(index, self.predicted_states.size());
            return ObjectFromStruct(self.predicted_states[index]);
          })
      .function(
          "toJson", +[](const OptimizationOutputs& self) { return json(self).dump(); });

  em::class_<Optimization>("Optimization")
      .constructor<OptimizationParams>()
      .function(
          "step",
          +[](Optimization& self, em::val state, em::val params) {
            return self.Step(StructFromObject<SingleCartPoleState>(state),
                             StructFromObject<SingleCartPoleParams>(params));
          })
      .function("reset", &Optimization::Reset);

#ifdef MINI_OPT_TRACING
  em::function(
      "getTraces", +[]() { return mini_opt::trace_collector::get_instance()->get_trace_json(); });
#endif

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
  em::function("doLeakCheck", &__lsan_do_recoverable_leak_check);
#endif
#endif
}

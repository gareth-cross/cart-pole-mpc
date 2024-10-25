// Copyright (c) 2024 Gareth Cross.
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OptimizationParams, control_dt, window_length, state_spacing,
                                   max_iterations, relative_exit_tol, absolute_first_derivative_tol,
                                   equality_penalty_initial, u_guess_sinusoid_amplitude, u_cost_weight,
                                   u_derivative_cost_weight, b_x_final_cost_weight);

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
      .constructor<>()
      .function(
          "step",
          +[](Simulator& sim, em::val params, double dt, double u, em::val f_external) {
            const auto external_forces = StructFromObject<std::vector<Vector2>>(f_external);
            F_ASSERT_EQ(2, external_forces.size());
            return sim.Step(StructFromObject<SingleCartPoleParams>(params), dt, u,
                            external_forces[0], external_forces[1]);
          })
      .function(
          "getState", +[](const Simulator& sim) { return ObjectFromStruct(sim.GetState()); })
      .function(
          "setState", +[](Simulator& sim, em::val state) {
            sim.SetState(StructFromObject<SingleCartPoleState>(state));
          });

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
      .constructor(+[](em::val params) {
        return Optimization(StructFromObject<OptimizationParams>(params));
      })
      .function(
          "step",
          +[](Optimization& self, em::val state, em::val params, double b_x_set_point) {
            return self.Step(StructFromObject<SingleCartPoleState>(state),
                             StructFromObject<SingleCartPoleParams>(params), b_x_set_point);
          })
      .function("reset", &Optimization::Reset);

  em::function(
      "getDefaultOptimizationParams", +[]() { return ObjectFromStruct(OptimizationParams{}); });

  em::function(
      "isTracingEnabled", +[]() {
#ifdef MINI_OPT_TRACING
        return true;
#else
    return false;
#endif
      });

  em::function(
      "getTraces", +[]() {
#ifdef MINI_OPT_TRACING
        return mini_opt::trace_collector::get_instance()->get_trace_json();
#else
    return std::string{};
#endif
      });

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
  em::function("doLeakCheck", &__lsan_do_recoverable_leak_check);
#endif
#endif
}

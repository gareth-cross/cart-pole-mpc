// Copyright (c) 2024 Gareth Cross.
// Wrapper code for our pendulum dynamics model.
#include <algorithm>
#include <chrono>
#include <execution>
#include <numeric>
#include <vector>

// Some macros to disable particular errors in third party imports.
#if defined(__GNUC__)  // gcc
#define BEGIN_THIRD_PARTY_INCLUDES                                \
  _Pragma("GCC diagnostic push")                       /* push */ \
      _Pragma("GCC diagnostic ignored \"-Wpedantic\"") /* disable pedantic */
#define END_THIRD_PARTY_INCLUDES _Pragma("GCC diagnostic pop")

#elif defined(__clang__)  // clang
#define BEGIN_THIRD_PARTY_INCLUDES                                  \
  _Pragma("clang diagnostic push")                       /* push */ \
      _Pragma("clang diagnostic ignored \"-Wpedantic\"") /* disable pedantic */
#define END_THIRD_PARTY_INCLUDES _Pragma("clang diagnostic pop")
#else
#define BEGIN_THIRD_PARTY_INCLUDES
#define END_THIRD_PARTY_INCLUDES
#endif

BEGIN_THIRD_PARTY_INCLUDES
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
END_THIRD_PARTY_INCLUDES

#include <mini_opt/assertions.hpp>

#include "optimization.hpp"
#include "simulator.hpp"

namespace nb = nanobind;
namespace pendulum {

void wrap_everything(nb::module_& m) {
  nb::class_<SingleCartPoleParams>(m, "SingleCartPoleParams")
      .def(nb::init<>())
      .def(nb::init<double, double, double, double, double, double, double, double, double>(),
           nb::arg("m_b"), nb::arg("m_1"), nb::arg("l_1"), nb::arg("g"), nb::arg("mu_b"),
           nb::arg("v_mu_b"), nb::arg("c_d_1"), nb::arg("x_s"), nb::arg("k_s"))
      .def_rw("m_b", &SingleCartPoleParams::m_b)
      .def_rw("m_1", &SingleCartPoleParams::m_1)
      .def_rw("l_1", &SingleCartPoleParams::l_1)
      .def_rw("g", &SingleCartPoleParams::g)
      .def_rw("mu_b", &SingleCartPoleParams::mu_b)
      .def_rw("v_mu_b", &SingleCartPoleParams::v_mu_b)
      .def_rw("c_d_1", &SingleCartPoleParams::c_d_1)
      .def_rw("x_s", &SingleCartPoleParams::x_s)
      .def_rw("k_s", &SingleCartPoleParams::k_s);

  // Wrap the optimization:
  nb::class_<OptimizationParams>(m, "OptimizationParams")
      .def(nb::init<>())
      .def_rw("control_dt", &OptimizationParams::control_dt)
      .def_rw("window_length", &OptimizationParams::window_length)
      .def_rw("state_spacing", &OptimizationParams::state_spacing)
      .def_rw("max_iterations", &OptimizationParams::max_iterations)
      .def_rw("relative_exit_tol", &OptimizationParams::relative_exit_tol)
      .def_rw("absolute_first_derivative_tol", &OptimizationParams::absolute_first_derivative_tol)
      .def_rw("equality_penalty_initial", &OptimizationParams::equality_penalty_initial)
      .def_rw("u_guess_sinusoid_amplitude", &OptimizationParams::u_guess_sinusoid_amplitude)
      .def_rw("u_cost_weight", &OptimizationParams::u_cost_weight)
      .def_rw("u_derivative_cost_weight", &OptimizationParams::u_derivative_cost_weight)
      .def_rw("b_x_final_cost_weight", &OptimizationParams::b_x_final_cost_weight);

  nb::class_<SingleCartPoleState>(m, "SingleCartPoleState")
      .def(nb::init<double, double, double, double>())
      .def_rw("b_x", &SingleCartPoleState::b_x)
      .def_rw("th_1", &SingleCartPoleState::th_1)
      .def_rw("b_x_dot", &SingleCartPoleState::b_x_dot)
      .def_rw("th_1_dot", &SingleCartPoleState::th_1_dot);

  nb::class_<OptimizationOutputs>(m, "OptimizationOutputs")
      .def("solver_summary",
           [](const OptimizationOutputs& self) { return self.solver_outputs.ToString(); })
      .def_ro("u", &OptimizationOutputs::u)
      .def_ro("predicted_states", &OptimizationOutputs::predicted_states);

  nb::class_<Optimization>(m, "Optimization")
      .def(nb::init<const OptimizationParams&>())
      .def("step", &Optimization::Step)
      .def("set_previous_solution", &Optimization::SetPreviousSolution);

  nb::class_<Vector2>(m, "Vector2").def(nb::init<double, double>());

  nb::class_<Simulator>(m, "Simulator")
      .def(nb::init<>())
      .def("step", &Simulator::Step)
      .def("get_state", &Simulator::GetState);
}

}  // namespace pendulum

NB_MODULE(PY_MODULE_NAME, m) { pendulum::wrap_everything(m); }

// Wrapper code for our pendulum dynamics model.
// Copyright 2024 Gareth Cross.
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
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
END_THIRD_PARTY_INCLUDES

#include <mini_opt/assertions.hpp>

#include "double_pendulum_dynamics.hpp"
#include "integration.hpp"
#include "optimization.hpp"
#include "single_pendulum_dynamics.hpp"

namespace nb = nanobind;
namespace pendulum {

// We need to evaluate the forward dynamics model over a set of timesteps.
// Pendulum params. (B, ) of these.
// Timestep size: `dt`, 1 scalar.
// Inputs: [u(0), u(1), ..., u(N - 1)], (B, N) of these.
// Initial state: x(0), (B, D) of these. Columns: [b_x, th_1, th_2, b_x', th_1', th_2']
//
// `D` is the state dimension (6 for double, 4 for single).
//
// We will return a:
//  - (B, N, D) array of states over the planning horizon.
//  - (B, N, N, D) array of derivatives of states wrt the control inputs.
#if 0
template <typename P, std::size_t D>
auto evaluate_forward_dynamics(
    const std::vector<P>& params, const double dt,
    const nb::ndarray<const double, nb::shape<-1, -1>, nb::device::cpu> u_array,
    const nb::ndarray<const double, nb::shape<-1, D>, nb::device::cpu> x0_array) {
  // Check that the batch dimensions all match.
  const std::size_t B = u_array.shape(0);

  F_ASSERT_EQ(B, params.size(), "Wrong size for params: {}", params.size());
  F_ASSERT_EQ(B, x0_array.shape(0), "Wrong shape for x0: [{}, {}]", x0_array.shape(0),
              x0_array.shape(1));

  // https://nanobind.readthedocs.io/en/latest/ndarray.html#fast-array-views
  const auto u_view = u_array.view();
  const auto x0_view = x0_array.view();

  // If there are `N` control inputs, we'll integrate them all and produce `N` outputs.
  const std::size_t N = u_view.shape(1);
  F_ASSERT_GT(N, 0);

  // Stores the returned data. We allocate this with `new` and the capsule takes ownership of it.
  // https://nanobind.readthedocs.io/en/latest/ndarray.html#fast-array-views
  struct data_storage {
    explicit data_storage(std::size_t batch_size, std::size_t num_states, std::size_t state_dim)
        : x_data(batch_size * num_states * state_dim),
          x_D_u_data(batch_size * num_states * num_states * state_dim),
          x_D_x0_data(batch_size * num_states * state_dim * state_dim) {}

    std::vector<double> x_data;
    std::vector<double> x_D_u_data;   // [B, N, N, D] derivative of all output `X` wrt input `U`.
    std::vector<double> x_D_x0_data;  // [B, N, D, D] derivative of all output `X` wrt initial `X0`.
  };
  data_storage* const storage = new data_storage(B, N, D);

  // `capsule` will own the storage:
  nb::capsule capsule(static_cast<const void*>(storage),
                      [](void* p) noexcept { delete static_cast<const data_storage*>(p); });

  // Create spans over the output arrays.
  constexpr wf::constant<D> _D{};
  constexpr wf::constant<1> _1{};
  const auto x_out = wf::make_span(storage->x_data.data(), wf::make_value_pack(B, N, _D),
                                   wf::make_value_pack(N * D, _D, _1));
  const auto x_D_u_out = wf::make_span(storage->x_D_u_data.data(), wf::make_value_pack(B, N, N, _D),
                                       wf::make_value_pack(N * N * D, N * D, _D, _1));
  const auto x_D_x0_out =
      wf::make_span(storage->x_D_x0_data.data(), wf::make_value_pack(B, N, _D, _D),
                    wf::make_value_pack(N * D * D, D * D, _D, _1));

  // Iterate over batch elements:
  for (std::size_t b = 0; b < B; ++b) {
    // Read the initial state from `x0`:
    Eigen::Matrix<double, D, 1> x;
    for (std::size_t d = 0; d < D; ++d) {
      x[d] = x0_view(b, d);
    }

    // Storage for jacobians of our integration method.
    std::vector<Eigen::Matrix<double, D, D>> f_D_x(N);
    std::vector<Eigen::Matrix<double, D, 1>> f_D_u(N);

    // Integrate forward.
    for (std::size_t i = 0; i < N; ++i) {
      // Integrate `x` and compute derivatives.
      if constexpr (D == 6) {
        std::tie(x, f_D_x[i], f_D_u[i]) = runge_kutta_4th_order(
            x, static_cast<double>(u_view(b, i)), dt,
            [&](const Eigen::Matrix<double, D, 1>& x_updated, const double u,
                Eigen::Matrix<double, D, D>& x_dot_D_x, Eigen::Matrix<double, D, 1>& x_dot_D_u) {
              Eigen::Matrix<double, D, 1> x_dot;
              gen::double_pendulum_dynamics(params[b], x_updated, u, x_dot, x_dot_D_x, x_dot_D_u);
              return x_dot;
            });
        x[1] = mod_pi(x[1]);
        x[2] = mod_pi(x[2]);
      } else {
        std::tie(x, f_D_x[i], f_D_u[i]) = runge_kutta_4th_order(
            x, static_cast<double>(u_view(b, i)), dt,
            [&](const Eigen::Matrix<double, D, 1>& x_updated, const double u,
                Eigen::Matrix<double, D, D>& x_dot_D_x, Eigen::Matrix<double, D, 1>& x_dot_D_u) {
              Eigen::Matrix<double, D, 1> x_dot;
              gen::single_pendulum_dynamics(params[b], x_updated, u, x_dot, x_dot_D_x, x_dot_D_u);
              return x_dot;
            });
        x[1] = mod_pi(x[1]);
      }

      for (std::size_t dim = 0; dim < D; ++dim) {
        F_ASSERT_LT(static_cast<std::size_t>(x_out.compute_index(b, i, dim)),
                    storage->x_data.size());
        x_out(b, i, dim) = x[dim];
      }
    }

    // Now we do the derivative computation. We have:
    //  x(k + 1) = rk4(x_k, u_k)
    //
    // So:
    //  dx(k+1)/dx(k) = d[rk4(x_k, u_k)]/dx_k
    //  dx(k+1)/du(k) = d[rk4(x_k, u_k)]/du_k
    //
    // We can chain rule this over time to compute the jacobian of any timestep wrt the control
    // inputs that came before it. This is similar to the propagation in a direct-shooting
    // approach.
    //
    // We iterate with `i` over the control inputs, u(i):
    for (std::size_t i = 0; i < N; ++i) {
      Eigen::Matrix<double, D, 1> f_D_u_i = f_D_u[i];

      // We iterate with `j` over the output states, x(j).
      // Note that x(j) is the state _after_ the integration of u(j).
      for (std::size_t j = i; j < N; ++j) {
        // Copy to the output:
        for (std::size_t dim = 0; dim < D; ++dim) {
          F_ASSERT_LT(static_cast<std::size_t>(x_D_u_out.compute_index(b, j, i, dim)),
                      storage->x_D_u_data.size(),
                      "b = {}, j = {}, i = {}, dim = {}, B = {}, N = {}", b, j, i, dim, B, N);
          x_D_u_out(b, j, i, dim) = f_D_u_i[dim];
        }
        // Propagate forward derivative of the output state wrt our control input at time `i`.
        f_D_u_i = (f_D_x[j] * f_D_u_i).eval();
      }
    }

    // Iterate over states to get the total derivative wrt the input state:
    Eigen::Matrix<double, D, D> x_i_D_x0 = Eigen::Matrix<double, D, D>::Identity();
    for (std::size_t i = 0; i < N; ++i) {
      x_i_D_x0 = (f_D_x[i] * x_i_D_x0).eval();
      // TODO: Could do this by assigning directly to an Eigen::Map...
      for (std::size_t row = 0; row < D; ++row) {
        for (std::size_t col = 0; col < D; ++col) {
          F_ASSERT_LT(static_cast<std::size_t>(x_D_x0_out.compute_index(b, i, row, col)),
                      storage->x_D_x0_data.size(),
                      "b = {}, i = {}, row = {}, col = {}, N = {}, D = {}", b, i, row, col, N, D);
          x_D_x0_out(b, i, row, col) = x_i_D_x0(row, col);
        }
      }
    }
  }  //  End of iteration over batch.

  return nb::make_tuple(
      nb::ndarray<nb::numpy, const double>(x_out.data(), {B, N, D}, capsule),
      nb::ndarray<nb::numpy, const double>(x_D_u_out.data(), {B, N, N, D}, capsule),
      nb::ndarray<nb::numpy, const double>(x_D_x0_out.data(), {B, N, D, D}, capsule));
}
#endif

void wrap_everything(nb::module_& m) {
  nb::class_<SingleCartPoleParams>(m, "SingleCartPoleParams")
      .def(nb::init<>())
      .def(nb::init<double, double, double, double>(), nb::arg("m_b"), nb::arg("m_1"),
           nb::arg("l_1"), nb::arg("g"))
      .def_rw("m_b", &SingleCartPoleParams::m_b)
      .def_rw("m_1", &SingleCartPoleParams::m_1)
      .def_rw("l_1", &SingleCartPoleParams::l_1)
      .def_rw("g", &SingleCartPoleParams::g);

#if 0
  m.def("evaluate_forward_dynamics_double", &evaluate_forward_dynamics<DoubleCartPoleParams, 6>,
        nb::arg("params"), nb::arg("dt"), nb::arg("u"), nb::arg("x0"));
  m.def("evaluate_forward_dynamics_single", &evaluate_forward_dynamics<SingleCartPoleParams, 4>,
        nb::arg("params"), nb::arg("dt"), nb::arg("u"), nb::arg("x0"));
#endif

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
      .def_rw("u_penalty", &OptimizationParams::u_penalty)
      .def_rw("u_derivative_penalty", &OptimizationParams::u_derivative_penalty)
      .def_rw("b_x_final_penalty", &OptimizationParams::b_x_final_penalty)
      .def_rw("terminal_angle_constraint_enabled", &OptimizationParams::terminal_angle_constraint_enabled);

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
}

}  // namespace pendulum

NB_MODULE(PY_MODULE_NAME, m) { pendulum::wrap_everything(m); }

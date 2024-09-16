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
#include <nanobind/stl/vector.h>
END_THIRD_PARTY_INCLUDES

#include "assertions.hpp"

#define WF_SPAN_EIGEN_SUPPORT
#include "dynamics.hpp"

namespace nb = nanobind;
namespace pendulum {

// We need to evaluate the forward dynamics model over a set of timesteps.
// Pendulum params. (B, ) of these.
// Timestep size: `dt`, 1 scalar.
// Inputs: [u(0), u(1), ..., u(N - 1)], (B, N) of these.
// Initial state: x(0), (B, 6) of these. Columns: [b_x, th_1, th_2, b_x', th_1', th_2']
//
// We will return a:
//  - (B, N, 6) array of states over the planning horizon.
//  - (B, N, N, 6) array of derivatives of states wrt the control inputs.
auto evaluate_forward_dynamics(
    const std::vector<pendulum::PendulumParams>& params, const double dt,
    const nb::ndarray<const double, nb::shape<-1, -1>, nb::device::cpu> u_array,
    const nb::ndarray<const double, nb::shape<-1, 6>, nb::device::cpu> x0_array) {
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

  // Stores the returned data. We place this in a unique_ptr so it can be moved into the capsule.
  // https://nanobind.readthedocs.io/en/latest/ndarray.html#fast-array-views
  struct data_storage {
    explicit data_storage(std::size_t batch_size, std::size_t num_states)
        : x_data(batch_size * num_states * 6),
          x_D_u_data(batch_size * num_states * num_states * 6) {}

    std::vector<double> x_data;
    std::vector<double> x_D_u_data;
  };
  data_storage* const storage = new data_storage(B, N);

  // `capsule` will own the storage:
  nb::capsule capsule(static_cast<const void*>(storage),
                      [](void* p) noexcept { delete static_cast<const data_storage*>(p); });

  // Create spans over the output arrays.
  const auto x_out =
      wf::make_span(storage->x_data.data(), wf::make_value_pack(B, N, wf::constant<6>{}),
                    wf::make_value_pack(N * 6, 6, wf::constant<1>{}));
  const auto x_D_u_out =
      wf::make_span(storage->x_D_u_data.data(), wf::make_value_pack(B, N, N, wf::constant<6>{}),
                    wf::make_value_pack(N * N * 6, N * 6, 6, wf::constant<1>{}));

  // Iterate over batch elements:
  for (std::size_t b = 0; b < B; ++b) {
    // Read the initial state from `x0`:
    Eigen::Matrix<double, 6, 1> x = (Eigen::Matrix<double, 6, 1>() << x0_view(b, 0), x0_view(b, 1),
                                     x0_view(b, 2), x0_view(b, 3), x0_view(b, 4), x0_view(b, 5))
                                        .finished();

    // Storage for jacobians of f(x, u) wrt x and u, where d(x)/dt = f(x, u)
    std::vector<Eigen::Matrix<double, 6, 6>> f_D_x(N);
    std::vector<Eigen::Matrix<double, 6, 1>> f_D_u(N);

    // Integrate forward.
    // This is just a simple euler integration, but our horizon is not that long so it might be
    // ok?
    for (std::size_t i = 0; i < N; ++i) {
      Eigen::Matrix<double, 6, 1> x_dot;
      gen::pendulum_dynamics(params[b], x, static_cast<double>(u_view(b, i)), x_dot, f_D_x[i],
                             f_D_u[i]);
      x.noalias() += x_dot * dt;

      for (std::size_t dim = 0; dim < 6; ++dim) {
        F_ASSERT_LT(x_out.compute_index(b, i, dim), storage->x_data.size());
        x_out(b, i, dim) = x[dim];
      }
    }

    // Now we do the derivative computation. We have:
    //  x(k + 1) = x(k) + f(x_k, u_k) * dt
    //
    // So:
    //  dx(k+1)/dx(k) = Identity + f_D_x * dt
    //  dx(k+1)/du(k) = f_D_u * dt
    //
    // We can chain rule this over time to compute the jacobian of any timestep wrt the control
    // inputs that came before it. This is similar to the propagation in a direct-shooting
    // approach.
    //
    // We iterate with `i` over the control inputs, u(i):
    for (std::size_t i = 0; i < N; ++i) {
      Eigen::Matrix<double, 6, 1> f_D_u_i = f_D_u[i] * dt;

      // We iterate with `j` over the output states, x(j).
      // Note that x(j) is the state _after_ the integration of u(j).
      for (std::size_t j = i; j < N; ++j) {
        // Copy to the output:
        for (std::size_t dim = 0; dim < 6; ++dim) {
          F_ASSERT_LT(x_D_u_out.compute_index(b, j, i, dim), storage->x_D_u_data.size(),
                      "b = {}, j = {}, i = {}, dim = {}, B = {}, N = {}", b, j, i, dim, B, N);
          x_D_u_out(b, j, i, dim) = f_D_u_i[dim];
        }
        // Propagate forward derivative of the output state wrt our control input at time `i`.
        f_D_u_i = ((Eigen::Matrix<double, 6, 6>::Identity() + f_D_x[j] * dt) * f_D_u_i).eval();
      }
    }
  }  //  End of iteration over batch.

  return nb::make_tuple(
      nb::ndarray<nb::numpy, const double>(x_out.data(), {B, N, 6}, capsule),
      nb::ndarray<nb::numpy, const double>(x_D_u_out.data(), {B, N, N, 6}, capsule));
}

}  // namespace pendulum

NB_MODULE(PY_MODULE_NAME, m) {
  nb::class_<pendulum::PendulumParams>(m, "PendulumParams")
      .def(nb::init<>())
      .def_rw("m_b", &pendulum::PendulumParams::m_b)
      .def_rw("m_1", &pendulum::PendulumParams::m_1)
      .def_rw("m_2", &pendulum::PendulumParams::m_2)
      .def_rw("l_1", &pendulum::PendulumParams::l_1)
      .def_rw("l_2", &pendulum::PendulumParams::l_2)
      .def_rw("g", &pendulum::PendulumParams::g);

  m.def("evaluate_forward_dynamics", &pendulum::evaluate_forward_dynamics, nb::arg("params"),
        nb::arg("dt"), nb::arg("u"), nb::arg("x0"));
}

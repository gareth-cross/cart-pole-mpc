// Machine generated code.
#pragma once
#include <cmath>
#include <cstdint>

#include <wrenfold/span.h>

// User-specified imports:
#include "structs.hpp"

namespace gen {

template <typename Scalar, typename T1, typename T3, typename T4, typename T5, typename T6,
          typename T7>
void single_pendulum_dynamics(const pendulum::SingleCartPoleParams& params, const T1& x,
                              const Scalar u, const T3& f_base, const T4& f_mass, T5&& x_dot,
                              T6&& J_x, T7&& J_u) {
  auto _x = wf::make_input_span<4, 1>(x);
  auto _f_base = wf::make_input_span<2, 1>(f_base);
  auto _f_mass = wf::make_input_span<2, 1>(f_mass);
  auto _x_dot = wf::make_output_span<4, 1>(x_dot);
  auto _J_x = wf::make_optional_output_span<4, 4>(J_x);
  auto _J_u = wf::make_optional_output_span<4, 1>(J_u);

  // Operation counts:
  // add: 19
  // branch: 2
  // call: 3
  // divide: 5
  // multiply: 68
  // negate: 7
  // total: 104

  const Scalar v0008 = params.m_1;
  const Scalar v0016 = params.g;
  const Scalar v0020 = _x(1, 0);
  const Scalar v0001 = _x(3, 0);
  const Scalar v0009 = params.m_b + v0008;
  const Scalar v0013 = static_cast<Scalar>(1) / params.v_mu_b;
  const Scalar v0000 = _x(2, 0);
  const Scalar v0033 = _f_mass(1, 0);
  const Scalar v0025 = std::sin(v0020);
  const Scalar v0022 = params.l_1;
  const Scalar v0021 = std::cos(v0020);
  const Scalar v0236 = -v0025;
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0156 = v0008 * v0025;
  const Scalar v0225 = v0022 * (v0001 * v0008);
  const Scalar v0224 = (v0009 * params.mu_b) * -v0016;
  const Scalar v0015 = std::tanh(v0000 * v0013);
  const Scalar v0228 = v0156 * v0236;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0121 = v0004 * v0236 + v0021 * (v0033 + v0016 * -v0008);
  const Scalar v0036 = static_cast<Scalar>(1) / v0022;
  const Scalar v0081 = v0009 + v0228;
  const Scalar v0024 = v0002 + v0003 + v0004 + v0015 * v0224 + (v0001 * v0021) * v0225;
  const Scalar v0226 = (static_cast<Scalar>(1) / (v0022 * v0022)) * (v0009 * v0022);
  const Scalar v0217 = v0024 * v0036;
  const Scalar v0029 = static_cast<Scalar>(1) / (v0008 * v0081);
  const Scalar v0088 = v0024 + (v0025 * v0036) * (v0022 * v0121);
  const Scalar v0157 = v0008 * v0029;
  const Scalar v0092 = v0156 * v0217 + v0121 * v0226;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0169 = v0001 * v0001;
    const Scalar v0125 = v0021 * -v0004 + v0025 * (v0008 * v0016 + -v0033);
    const Scalar v0233 =
        ((static_cast<Scalar>(1) / (v0081 * v0081 * (v0008 * v0008))) * static_cast<Scalar>(2)) *
        (v0156 * (v0008 * v0021));
    const Scalar v0063 = static_cast<Scalar>(1) + v0015 * -v0015;
    const Scalar v0231 = v0157 * (v0021 * static_cast<Scalar>(2));
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) =
        v0008 *
        (v0029 * ((v0025 * v0125 + v0021 * v0121) * (v0022 * v0036) + -(v0022 * (v0156 * v0169))) +
         v0088 * v0233);
    _J_x(2, 2) = v0157 * (v0013 * v0063) * v0224;
    _J_x(2, 3) = v0225 * v0231;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) = v0029 * (v0125 * v0226 + v0008 * (v0169 * v0228 + v0021 * v0217)) + v0092 * v0233;
    _J_x(3, 2) = v0036 * v0063 * v0156 * (v0013 * v0029) * v0224;
    _J_x(3, 3) = v0001 * v0156 * v0231;
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0157;
    _J_u(3, 0) = v0156 * (v0029 * v0036);
  }
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0088 * v0157;
  _x_dot(3, 0) = v0029 * v0092;
}

}  // namespace gen

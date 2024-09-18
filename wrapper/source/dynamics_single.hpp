// Machine generated code.
#pragma once
#include <cmath>
#include <cstdint>

#include <wrenfold/span.h>

// User-specified imports:
#include "parameters.hpp"

namespace gen {

template <typename Scalar, typename T1, typename T3, typename T4, typename T5>
void single_pendulum_dynamics(const pendulum::PendulumParams& params, const T1& x, const Scalar u,
                              T3&& x_dot, T4&& J_x, T5&& J_u) {
  auto _x = wf::make_input_span<4, 1>(x);
  auto _x_dot = wf::make_output_span<4, 1>(x_dot);
  auto _J_x = wf::make_optional_output_span<4, 4>(J_x);
  auto _J_u = wf::make_optional_output_span<4, 1>(J_u);

  // Operation counts:
  // add: 11
  // branch: 2
  // call: 2
  // divide: 3
  // multiply: 49
  // negate: 4
  // total: 71

  const Scalar v0003 = _x(1, 0);
  const Scalar v0001 = _x(3, 0);
  const Scalar v0016 = std::cos(v0003);
  const Scalar v0006 = params.m_1;
  const Scalar v0004 = std::sin(v0003);
  const Scalar v0126 = v0006 * v0016;
  const Scalar v0020 = params.l_1;
  const Scalar v0124 = v0004 * v0006;
  const Scalar v0019 = u;
  const Scalar v0017 = params.g;
  const Scalar v0010 = v0006 + params.m_b;
  const Scalar v0022 = v0019 + v0020 * v0126 * (v0001 * v0001);
  const Scalar v0171 = -v0016;
  const Scalar v0058 = v0010 + v0124 * -v0004;
  const Scalar v0025 = static_cast<Scalar>(1) / v0020;
  const Scalar v0015 = static_cast<Scalar>(1) / (v0006 * v0058);
  const Scalar v0162 = v0025 * ((v0010 * v0017) * v0171 + v0004 * v0022);
  const Scalar v0125 = v0006 * v0015;
  const Scalar v0061 = v0022 + v0017 * v0124 * v0171;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0136 = v0001 * v0020;
    const Scalar v0128 = v0001 * v0004;
    const Scalar v0167 =
        ((static_cast<Scalar>(1) / (v0058 * v0058 * (v0006 * v0006))) * static_cast<Scalar>(2)) *
        (v0124 * v0126);
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = v0006 * (v0061 * v0167 +
                          (v0017 * (v0016 * v0171 + v0004 * v0004) + -(v0128 * v0136)) * v0125);
    _J_x(2, 2) = static_cast<Scalar>(0);
    _J_x(2, 3) = static_cast<Scalar>(2) * v0125 * v0126 * v0136;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v0006 * (v0162 * v0167 + v0015 * (v0124 * v0128 * -v0001 +
                                          v0025 * (v0016 * v0022 + v0010 * (v0004 * v0017))));
    _J_x(3, 2) = static_cast<Scalar>(0);
    _J_x(3, 3) = v0001 * v0124 * v0125 * (v0016 * static_cast<Scalar>(2));
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0125;
    _J_u(3, 0) = v0124 * (v0015 * v0025);
  }
  const Scalar v0000 = _x(2, 0);
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0061 * v0125;
  _x_dot(3, 0) = v0125 * v0162;
}

}  // namespace gen

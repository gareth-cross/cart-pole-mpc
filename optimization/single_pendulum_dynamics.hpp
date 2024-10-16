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
  // add: 18
  // branch: 2
  // call: 3
  // divide: 4
  // multiply: 61
  // negate: 6
  // total: 94

  const Scalar v0007 = params.m_1;
  const Scalar v0010 = params.g;
  const Scalar v0014 = _x(1, 0);
  const Scalar v0001 = _x(3, 0);
  const Scalar v0000 = _x(2, 0);
  const Scalar v0031 = _f_mass(1, 0);
  const Scalar v0020 = std::sin(v0014);
  const Scalar v0151 = v0001 * v0007;
  const Scalar v0016 = params.l_1;
  const Scalar v0015 = std::cos(v0014);
  const Scalar v0200 = -v0020;
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0191 = v0016 * v0151;
  const Scalar v0008 = params.m_b + v0007;
  const Scalar v0150 = v0007 * v0020;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0115 = v0004 * v0200 + v0015 * (v0031 + v0010 * -v0007);
  const Scalar v0034 = static_cast<Scalar>(1) / v0016;
  const Scalar v0018 =
      v0002 + v0003 + v0004 +
      v0008 * v0010 * params.mu_b *
          static_cast<Scalar>((static_cast<Scalar>(0) < v0000) - (v0000 < static_cast<Scalar>(0))) +
      (v0001 * v0015) * v0191;
  const Scalar v0075 = v0008 + v0150 * v0200;
  const Scalar v0193 = (static_cast<Scalar>(1) / (v0016 * v0016)) * (v0008 * v0016);
  const Scalar v0185 = v0018 * v0034;
  const Scalar v0027 = static_cast<Scalar>(1) / (v0007 * v0075);
  const Scalar v0082 = v0018 + (v0020 * v0034) * (v0016 * v0115);
  const Scalar v0086 = v0150 * v0185 + v0115 * v0193;
  const Scalar v0192 = v0027 * v0150;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0119 = v0015 * -v0004 + v0020 * (v0007 * v0010 + -v0031);
    const Scalar v0153 = v0007 * v0015;
    const Scalar v0197 =
        ((static_cast<Scalar>(1) / (v0075 * v0075 * (v0007 * v0007))) * static_cast<Scalar>(2)) *
        (v0150 * v0153);
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = v0007 * (v0027 * ((v0020 * v0119 + v0015 * v0115) * (v0016 * v0034) +
                                   -(v0016 * v0150 * (v0001 * v0001))) +
                          v0082 * v0197);
    _J_x(2, 2) = static_cast<Scalar>(0);
    _J_x(2, 3) = v0153 * (v0027 * static_cast<Scalar>(2)) * v0191;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v0027 * (v0119 * v0193 + v0007 * (v0150 * (v0001 * v0020) * -v0001 + v0015 * v0185)) +
        v0086 * v0197;
    _J_x(3, 2) = static_cast<Scalar>(0);
    _J_x(3, 3) = v0151 * (v0015 * static_cast<Scalar>(2)) * v0192;
  }
  const Scalar v0154 = v0007 * v0027;
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0154;
    _J_u(3, 0) = v0034 * v0192;
  }
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0082 * v0154;
  _x_dot(3, 0) = v0027 * v0086;
}

}  // namespace gen

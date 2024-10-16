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
  // add: 17
  // branch: 2
  // call: 2
  // divide: 4
  // multiply: 58
  // negate: 6
  // total: 89

  const Scalar v0008 = params.m_1;
  const Scalar v0025 = params.g;
  const Scalar v0005 = _x(1, 0);
  const Scalar v0001 = _x(3, 0);
  const Scalar v0027 = _f_mass(1, 0);
  const Scalar v0013 = std::sin(v0005);
  const Scalar v0147 = v0001 * v0008;
  const Scalar v0009 = params.l_1;
  const Scalar v0006 = std::cos(v0005);
  const Scalar v0196 = -v0013;
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0187 = v0009 * v0147;
  const Scalar v0146 = v0008 * v0013;
  const Scalar v0017 = v0008 + params.m_b;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0111 = v0004 * v0196 + v0006 * (v0027 + v0025 * -v0008);
  const Scalar v0030 = static_cast<Scalar>(1) / v0009;
  const Scalar v0011 = v0002 + v0003 + v0004 + (v0001 * v0006) * v0187;
  const Scalar v0071 = v0017 + v0146 * v0196;
  const Scalar v0189 = (static_cast<Scalar>(1) / (v0009 * v0009)) * (v0009 * v0017);
  const Scalar v0180 = v0011 * v0030;
  const Scalar v0022 = static_cast<Scalar>(1) / (v0008 * v0071);
  const Scalar v0078 = v0011 + (v0013 * v0030) * (v0009 * v0111);
  const Scalar v0082 = v0146 * v0180 + v0111 * v0189;
  const Scalar v0188 = v0022 * v0146;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0115 = v0006 * -v0004 + v0013 * (v0008 * v0025 + -v0027);
    const Scalar v0148 = v0006 * v0008;
    const Scalar v0193 =
        ((static_cast<Scalar>(1) / (v0071 * v0071 * (v0008 * v0008))) * static_cast<Scalar>(2)) *
        (v0146 * v0148);
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = v0008 * (v0022 * ((v0013 * v0115 + v0006 * v0111) * (v0009 * v0030) +
                                   -(v0009 * v0146 * (v0001 * v0001))) +
                          v0078 * v0193);
    _J_x(2, 2) = static_cast<Scalar>(0);
    _J_x(2, 3) = v0148 * (v0022 * static_cast<Scalar>(2)) * v0187;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v0022 * (v0115 * v0189 + v0008 * (v0146 * (v0001 * v0013) * -v0001 + v0006 * v0180)) +
        v0082 * v0193;
    _J_x(3, 2) = static_cast<Scalar>(0);
    _J_x(3, 3) = v0147 * (v0006 * static_cast<Scalar>(2)) * v0188;
  }
  const Scalar v0150 = v0008 * v0022;
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0150;
    _J_u(3, 0) = v0030 * v0188;
  }
  const Scalar v0000 = _x(2, 0);
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0078 * v0150;
  _x_dot(3, 0) = v0022 * v0082;
}

}  // namespace gen

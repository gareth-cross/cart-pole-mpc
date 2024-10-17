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
  // add: 39
  // branch: 4
  // call: 4
  // compare: 1
  // divide: 6
  // multiply: 115
  // negate: 8
  // total: 177

  const Scalar v0022 = _x(1, 0);
  const Scalar v0026 = std::sin(v0022);
  const Scalar v0024 = params.l_1;
  const Scalar v0001 = _x(3, 0);
  const Scalar v0379 = -v0026;
  const Scalar v0294 = v0001 * v0024;
  const Scalar v0023 = std::cos(v0022);
  const Scalar v0000 = _x(2, 0);
  const Scalar v0325 = v0023 * v0023;
  const Scalar v0028 = v0000 + v0294 * v0379;
  const Scalar v0373 = v0294 * v0325;
  const Scalar v0030 = v0294 * v0373 + v0028 * v0028;
  const Scalar v0039 = std::sqrt(v0030);
  const Scalar v0312 = static_cast<Scalar>(1.5) * v0039;
  const Scalar v0303 = v0028 * static_cast<Scalar>(2);
  const Scalar v0302 = v0024 * static_cast<Scalar>(2);
  const Scalar v0230 = v0028 * v0379 + v0373;
  const bool v0032 = static_cast<Scalar>(0) < v0030;
  Scalar v0041;
  Scalar v0058;
  if (v0032) {
    v0041 = v0303 * v0312;
    v0058 = v0230 * v0302 * v0312;
  } else {
    v0041 = static_cast<Scalar>(0);
    v0058 = static_cast<Scalar>(0);
  }
  const Scalar v0008 = params.m_1;
  const Scalar v0016 = params.g;
  const Scalar v0060 = _f_mass(1, 0);
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0009 = params.m_b + v0008;
  const Scalar v0013 = static_cast<Scalar>(1) / params.v_mu_b;
  const Scalar v0296 = v0008 * v0026;
  const Scalar v0295 = (static_cast<Scalar>(-0.16666666666666666)) * params.c_d_1;
  const Scalar v0377 = (v0009 * params.mu_b) * -v0016;
  const Scalar v0015 = std::tanh(v0000 * v0013);
  const Scalar v0063 = static_cast<Scalar>(1) / v0024;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0321 = v0008 * (v0009 + v0296 * v0379);
  const Scalar v0309 = v0009 * (static_cast<Scalar>(1) / (v0024 * v0024));
  const Scalar v0240 = v0058 * v0295 + v0024 * (v0004 * v0379 + v0023 * (v0060 + v0016 * -v0008));
  const Scalar v0369 = v0063 * v0296;
  const Scalar v0045 =
      v0002 + v0003 + v0004 + v0015 * v0377 + v0041 * v0295 + v0008 * v0294 * (v0001 * v0023);
  const Scalar v0298 = v0026 * v0063;
  const Scalar v0049 = static_cast<Scalar>(1) / v0321;
  const Scalar v0368 = v0049 * v0063;
  const Scalar v0179 = v0045 * v0369 + v0240 * v0309;
  const Scalar v0175 = v0045 + v0240 * v0298;
  const Scalar v0301 = v0008 * v0049;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0383 = -v0023;
    Scalar v0092;
    Scalar v0105;
    Scalar v0114;
    Scalar v0123;
    Scalar v0135;
    if (v0032) {
      const Scalar v0084 = static_cast<Scalar>(1) / v0039;
      const Scalar v0079 = static_cast<Scalar>(0.75);
      const Scalar v0304 = static_cast<Scalar>(2) * v0079;
      const Scalar v0371 = v0304 * (v0084 * v0230);
      const Scalar v0242 = v0028 + v0026 * v0294;
      const Scalar v0364 = v0039 * static_cast<Scalar>(3);
      v0092 = -((v0294 * v0371 + v0312) * (v0023 * v0024) * (static_cast<Scalar>(2) * v0242));
      v0105 = (v0084 * v0242 * (v0303 * v0304) + v0364) * v0294 * v0383;
      v0114 = v0024 * (v0364 * v0379 + v0303 * v0371);
      v0123 = v0303 * v0303 * (v0079 * v0084) + v0364;
      v0135 = v0024 * (v0230 * v0371 + (v0325 + v0026 * v0026) * v0312) * v0302;
    } else {
      v0092 = static_cast<Scalar>(0);
      v0105 = static_cast<Scalar>(0);
      v0114 = static_cast<Scalar>(0);
      v0123 = static_cast<Scalar>(0);
      v0135 = static_cast<Scalar>(0);
    }
    const Scalar v0256 = v0092 * v0295 + v0024 * (v0004 * v0383 + v0026 * (v0008 * v0016 + -v0060));
    const Scalar v0107 = v0294 * v0296 * -v0001 + v0105 * v0295;
    const Scalar v0307 = v0008 * v0023;
    const Scalar v0370 = v0114 * v0295;
    const Scalar v0074 = static_cast<Scalar>(1) / (v0321 * v0321);
    const Scalar v0139 = static_cast<Scalar>(2) * v0294 * v0307 + v0370;
    const Scalar v0372 = v0135 * v0295;
    const Scalar v0125 = v0013 * (static_cast<Scalar>(1) + v0015 * -v0015) * v0377 + v0123 * v0295;
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = v0008 * (v0175 * v0296 * v0307 * (static_cast<Scalar>(2) * v0074) +
                          v0049 * (v0107 + v0063 * (v0026 * v0256 + v0023 * v0240)));
    _J_x(2, 2) = (v0125 + v0298 * v0370) * v0301;
    _J_x(2, 3) = (v0139 + v0298 * v0372) * v0301;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v0049 * v0256 * v0309 + v0008 * (v0074 * v0179 * v0296 * (v0023 * static_cast<Scalar>(2)) +
                                         (v0023 * v0045 + v0026 * v0107) * v0368);
    _J_x(3, 2) = v0049 * (v0309 * v0370 + v0125 * v0369);
    _J_x(3, 3) = v0049 * (v0309 * v0372 + v0139 * v0369);
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0301;
    _J_u(3, 0) = v0296 * v0368;
  }
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0175 * v0301;
  _x_dot(3, 0) = v0049 * v0179;
}

}  // namespace gen

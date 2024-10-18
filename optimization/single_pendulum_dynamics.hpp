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
  // add: 44
  // branch: 9
  // call: 4
  // compare: 4
  // divide: 6
  // multiply: 119
  // negate: 12
  // total: 198

  const Scalar v0011 = params.v_mu_b;
  Scalar v0013;
  if (static_cast<Scalar>(1e-06) < v0011) {
    v0013 = v0011;
  } else {
    v0013 = static_cast<Scalar>(1e-06);
  }
  const Scalar v0025 = params.x_s;
  const Scalar v0024 = _x(0, 0);
  const Scalar v0027 = v0024 + -v0025;
  const bool v0029 = static_cast<Scalar>(0) < v0027;
  Scalar v0030;
  if (v0029) {
    v0030 = v0027;
  } else {
    v0030 = static_cast<Scalar>(0);
  }
  const Scalar v0420 = -(v0024 + v0025);
  const bool v0058 = static_cast<Scalar>(0) < v0420;
  Scalar v0059;
  if (v0058) {
    v0059 = v0420;
  } else {
    v0059 = static_cast<Scalar>(0);
  }
  const Scalar v0034 = _x(1, 0);
  const Scalar v0038 = std::sin(v0034);
  const Scalar v0036 = params.l_1;
  const Scalar v0001 = _x(3, 0);
  const Scalar v0424 = -v0038;
  const Scalar v0328 = v0001 * v0036;
  const Scalar v0035 = std::cos(v0034);
  const Scalar v0000 = _x(2, 0);
  const Scalar v0359 = v0035 * v0035;
  const Scalar v0040 = v0000 + v0328 * v0424;
  const Scalar v0413 = v0328 * v0359;
  const Scalar v0042 = v0328 * v0413 + v0040 * v0040;
  const Scalar v0050 = std::sqrt(v0042);
  const Scalar v0346 = static_cast<Scalar>(1.5) * v0050;
  const Scalar v0338 = v0040 * static_cast<Scalar>(2);
  const Scalar v0337 = v0036 * static_cast<Scalar>(2);
  const Scalar v0264 = v0040 * v0424 + v0413;
  const bool v0043 = static_cast<Scalar>(0) < v0042;
  Scalar v0052;
  Scalar v0074;
  if (v0043) {
    v0052 = v0338 * v0346;
    v0074 = v0264 * v0337 * v0346;
  } else {
    v0052 = static_cast<Scalar>(0);
    v0074 = static_cast<Scalar>(0);
  }
  const Scalar v0008 = params.m_1;
  const Scalar v0019 = params.g;
  const Scalar v0076 = _f_mass(1, 0);
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0009 = params.m_b + v0008;
  const Scalar v0016 = static_cast<Scalar>(1) / v0013;
  const Scalar v0330 = v0008 * v0038;
  const Scalar v0329 = (static_cast<Scalar>(-0.16666666666666666)) * params.c_d_1;
  const Scalar v0031 = params.k_s;
  const Scalar v0418 = (v0009 * params.mu_b) * -v0019;
  const Scalar v0018 = std::tanh(v0000 * v0016);
  const Scalar v0079 = static_cast<Scalar>(1) / v0036;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0355 = v0008 * (v0009 + v0330 * v0424);
  const Scalar v0343 = v0009 * (static_cast<Scalar>(1) / (v0036 * v0036));
  const Scalar v0274 = v0074 * v0329 + v0036 * (v0004 * v0424 + v0035 * (v0076 + v0019 * -v0008));
  const Scalar v0409 = v0079 * v0330;
  const Scalar v0193 = v0002 + v0003 + v0004 + v0018 * v0418 + v0052 * v0329 +
                       v0008 * v0328 * (v0001 * v0035) + v0031 * (v0059 + -v0030);
  const Scalar v0331 = v0038 * v0079;
  const Scalar v0065 = static_cast<Scalar>(1) / v0355;
  const Scalar v0366 = v0065 * v0079;
  const Scalar v0210 = v0193 * v0409 + v0274 * v0343;
  const Scalar v0206 = v0193 + v0274 * v0331;
  const Scalar v0416 = v0330 * v0366;
  const Scalar v0332 = v0008 * v0065;
  if (static_cast<bool>(_J_x)) {
    std::int64_t v0089;
    if (v0029) {
      v0089 = 1;
    } else {
      v0089 = 0;
    }
    std::int64_t v0092;
    if (v0058) {
      v0092 = -1;
    } else {
      v0092 = 0;
    }
    const Scalar v0428 = -v0035;
    Scalar v0116;
    Scalar v0129;
    Scalar v0138;
    Scalar v0147;
    Scalar v0159;
    if (v0043) {
      const Scalar v0108 = static_cast<Scalar>(1) / v0050;
      const Scalar v0103 = static_cast<Scalar>(0.75);
      const Scalar v0339 = static_cast<Scalar>(2) * v0103;
      const Scalar v0411 = v0339 * (v0108 * v0264);
      const Scalar v0276 = v0040 + v0038 * v0328;
      const Scalar v0404 = v0050 * static_cast<Scalar>(3);
      v0116 = -((v0328 * v0411 + v0346) * (v0035 * v0036) * (static_cast<Scalar>(2) * v0276));
      v0129 = (v0108 * v0276 * (v0338 * v0339) + v0404) * v0328 * v0428;
      v0138 = v0036 * (v0404 * v0424 + v0338 * v0411);
      v0147 = v0338 * v0338 * (v0103 * v0108) + v0404;
      v0159 = v0036 * (v0264 * v0411 + (v0359 + v0038 * v0038) * v0346) * v0337;
    } else {
      v0116 = static_cast<Scalar>(0);
      v0129 = static_cast<Scalar>(0);
      v0138 = static_cast<Scalar>(0);
      v0147 = static_cast<Scalar>(0);
      v0159 = static_cast<Scalar>(0);
    }
    const Scalar v0290 = v0116 * v0329 + v0036 * (v0004 * v0428 + v0038 * (v0008 * v0019 + -v0076));
    const Scalar v0131 = v0328 * v0330 * -v0001 + v0129 * v0329;
    const Scalar v0342 = v0008 * v0035;
    const Scalar v0410 = v0138 * v0329;
    const Scalar v0098 = static_cast<Scalar>(1) / (v0355 * v0355);
    const Scalar v0163 = static_cast<Scalar>(2) * v0328 * v0342 + v0410;
    const Scalar v0412 = v0159 * v0329;
    const Scalar v0149 = v0016 * (static_cast<Scalar>(1) + v0018 * -v0018) * v0418 + v0147 * v0329;
    const Scalar v0386 = v0031 * (static_cast<Scalar>(v0092) + -static_cast<Scalar>(v0089));
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = v0332 * v0386;
    _J_x(2, 1) = v0008 * (v0206 * v0330 * v0342 * (static_cast<Scalar>(2) * v0098) +
                          v0065 * (v0131 + v0079 * (v0038 * v0290 + v0035 * v0274)));
    _J_x(2, 2) = (v0149 + v0331 * v0410) * v0332;
    _J_x(2, 3) = (v0163 + v0331 * v0412) * v0332;
    _J_x(3, 0) = v0386 * v0416;
    _J_x(3, 1) =
        v0065 * v0290 * v0343 + v0008 * (v0098 * v0210 * v0330 * (v0035 * static_cast<Scalar>(2)) +
                                         (v0035 * v0193 + v0038 * v0131) * v0366);
    _J_x(3, 2) = v0065 * (v0343 * v0410 + v0149 * v0409);
    _J_x(3, 3) = v0065 * (v0343 * v0412 + v0163 * v0409);
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0332;
    _J_u(3, 0) = v0416;
  }
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0206 * v0332;
  _x_dot(3, 0) = v0065 * v0210;
}

}  // namespace gen

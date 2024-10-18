// Machine generated code.
#pragma once
#include <cmath>
#include <cstdint>

#include <wrenfold/span.h>

// User-specified imports:
#include "structs.hpp"

namespace gen {

template <typename Scalar, typename T1, typename T3, typename T4, typename T5, typename T6>
Eigen::Matrix<Scalar, 4, 1> single_pendulum_dynamics(const pendulum::SingleCartPoleParams& params,
                                                     const T1& x, const Scalar u, const T3& f_base,
                                                     const T4& f_mass, T5&& J_x, T6&& J_u) {
  auto _x = wf::make_input_span<4, 1>(x);
  auto _f_base = wf::make_input_span<2, 1>(f_base);
  auto _f_mass = wf::make_input_span<2, 1>(f_mass);
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

  const Scalar v0007 = params.x_s;
  const Scalar v0005 = _x(0, 0);
  const Scalar v0010 = v0005 + -v0007;
  const bool v0011 = static_cast<Scalar>(0) < v0010;
  Scalar v0048;
  if (v0011) {
    v0048 = v0010;
  } else {
    v0048 = static_cast<Scalar>(0);
  }
  const Scalar v0420 = -(v0005 + v0007);
  const bool v0018 = static_cast<Scalar>(0) < v0420;
  Scalar v0071;
  if (v0018) {
    v0071 = v0420;
  } else {
    v0071 = static_cast<Scalar>(0);
  }
  const Scalar v0039 = params.v_mu_b;
  Scalar v0041;
  if (static_cast<Scalar>(1e-06) < v0039) {
    v0041 = v0039;
  } else {
    v0041 = static_cast<Scalar>(1e-06);
  }
  const Scalar v0023 = _x(1, 0);
  const Scalar v0024 = std::sin(v0023);
  const Scalar v0053 = params.l_1;
  const Scalar v0051 = _x(3, 0);
  const Scalar v0424 = -v0024;
  const Scalar v0329 = v0051 * v0053;
  const Scalar v0052 = std::cos(v0023);
  const Scalar v0037 = _x(2, 0);
  const Scalar v0361 = v0052 * v0052;
  const Scalar v0056 = v0037 + v0329 * v0424;
  const Scalar v0415 = v0329 * v0361;
  const Scalar v0058 = v0329 * v0415 + v0056 * v0056;
  const Scalar v0065 = std::sqrt(v0058);
  const Scalar v0346 = static_cast<Scalar>(1.5) * v0065;
  const Scalar v0338 = v0056 * static_cast<Scalar>(2);
  const Scalar v0337 = v0053 * static_cast<Scalar>(2);
  const Scalar v0264 = v0056 * v0424 + v0415;
  const bool v0059 = static_cast<Scalar>(0) < v0058;
  Scalar v0067;
  Scalar v0084;
  if (v0059) {
    v0067 = v0338 * v0346;
    v0084 = v0264 * v0337 * v0346;
  } else {
    v0067 = static_cast<Scalar>(0);
    v0084 = static_cast<Scalar>(0);
  }
  const Scalar v0025 = params.m_1;
  const Scalar v0045 = params.g;
  const Scalar v0086 = _f_mass(1, 0);
  const Scalar v0036 = _f_mass(0, 0);
  const Scalar v0028 = v0025 + params.m_b;
  const Scalar v0042 = static_cast<Scalar>(1) / v0041;
  const Scalar v0330 = v0024 * v0025;
  const Scalar v0328 = (static_cast<Scalar>(-0.16666666666666666)) * params.c_d_1;
  const Scalar v0013 = params.k_s;
  const Scalar v0418 = (v0028 * params.mu_b) * -v0045;
  const Scalar v0044 = std::tanh(v0037 * v0042);
  const Scalar v0089 = static_cast<Scalar>(1) / v0053;
  const Scalar v0035 = _f_base(0, 0);
  const Scalar v0034 = u;
  const Scalar v0357 = v0025 * (v0028 + v0330 * v0424);
  const Scalar v0344 = v0028 * (static_cast<Scalar>(1) / (v0053 * v0053));
  const Scalar v0274 = v0084 * v0328 + v0053 * (v0036 * v0424 + v0052 * (v0086 + v0045 * -v0025));
  const Scalar v0409 = v0089 * v0330;
  const Scalar v0199 = v0034 + v0035 + v0036 + v0044 * v0418 + v0067 * v0328 +
                       v0025 * v0329 * (v0051 * v0052) + v0013 * (v0071 + -v0048);
  const Scalar v0331 = v0024 * v0089;
  const Scalar v0031 = static_cast<Scalar>(1) / v0357;
  const Scalar v0358 = v0031 * v0089;
  const Scalar v0260 = v0199 * v0409 + v0274 * v0344;
  const Scalar v0256 = v0199 + v0274 * v0331;
  const Scalar v0416 = v0330 * v0358;
  const Scalar v0332 = v0025 * v0031;
  if (static_cast<bool>(_J_x)) {
    std::int64_t v0012;
    if (v0011) {
      v0012 = 1;
    } else {
      v0012 = 0;
    }
    std::int64_t v0019;
    if (v0018) {
      v0019 = -1;
    } else {
      v0019 = 0;
    }
    const Scalar v0428 = -v0052;
    Scalar v0106;
    Scalar v0119;
    Scalar v0128;
    Scalar v0137;
    Scalar v0149;
    if (v0059) {
      const Scalar v0098 = static_cast<Scalar>(1) / v0065;
      const Scalar v0093 = static_cast<Scalar>(0.75);
      const Scalar v0339 = static_cast<Scalar>(2) * v0093;
      const Scalar v0411 = v0339 * (v0098 * v0264);
      const Scalar v0276 = v0056 + v0024 * v0329;
      const Scalar v0405 = v0065 * static_cast<Scalar>(3);
      v0106 = -((v0329 * v0411 + v0346) * (v0052 * v0053) * (static_cast<Scalar>(2) * v0276));
      v0119 = (v0098 * v0276 * (v0338 * v0339) + v0405) * v0329 * v0428;
      v0128 = v0053 * (v0405 * v0424 + v0338 * v0411);
      v0137 = v0338 * v0338 * (v0093 * v0098) + v0405;
      v0149 = v0053 * (v0264 * v0411 + (v0361 + v0024 * v0024) * v0346) * v0337;
    } else {
      v0106 = static_cast<Scalar>(0);
      v0119 = static_cast<Scalar>(0);
      v0128 = static_cast<Scalar>(0);
      v0137 = static_cast<Scalar>(0);
      v0149 = static_cast<Scalar>(0);
    }
    const Scalar v0290 = v0106 * v0328 + v0053 * (v0036 * v0428 + v0024 * (v0025 * v0045 + -v0086));
    const Scalar v0121 = v0329 * v0330 * -v0051 + v0119 * v0328;
    const Scalar v0343 = v0025 * v0052;
    const Scalar v0410 = v0128 * v0328;
    const Scalar v0075 = static_cast<Scalar>(1) / (v0357 * v0357);
    const Scalar v0153 = static_cast<Scalar>(2) * v0329 * v0343 + v0410;
    const Scalar v0412 = v0149 * v0328;
    const Scalar v0139 = v0042 * (static_cast<Scalar>(1) + v0044 * -v0044) * v0418 + v0137 * v0328;
    const Scalar v0375 = v0013 * (static_cast<Scalar>(v0019) + -static_cast<Scalar>(v0012));
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = v0332 * v0375;
    _J_x(2, 1) = v0025 * (v0256 * v0330 * v0343 * (static_cast<Scalar>(2) * v0075) +
                          v0031 * (v0121 + v0089 * (v0024 * v0290 + v0052 * v0274)));
    _J_x(2, 2) = (v0139 + v0331 * v0410) * v0332;
    _J_x(2, 3) = (v0153 + v0331 * v0412) * v0332;
    _J_x(3, 0) = v0375 * v0416;
    _J_x(3, 1) =
        v0031 * v0290 * v0344 + v0025 * (v0075 * v0260 * v0330 * (v0052 * static_cast<Scalar>(2)) +
                                         (v0052 * v0199 + v0024 * v0121) * v0358);
    _J_x(3, 2) = v0031 * (v0344 * v0410 + v0139 * v0409);
    _J_x(3, 3) = v0031 * (v0344 * v0412 + v0153 * v0409);
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0332;
    _J_u(3, 0) = v0416;
  }
  return Eigen::Matrix<double, 4, 1>(v0037, v0051, v0256 * v0332, v0031 * v0260);
}

}  // namespace gen

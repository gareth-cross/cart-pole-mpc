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
  // branch: 5
  // call: 4
  // compare: 2
  // divide: 6
  // multiply: 115
  // negate: 8
  // total: 179

  const Scalar v0011 = params.v_mu_b;
  Scalar v0013;
  if (static_cast<Scalar>(1e-16) < v0011) {
    v0013 = v0011;
  } else {
    v0013 = static_cast<Scalar>(1e-16);
  }
  const Scalar v0025 = _x(1, 0);
  const Scalar v0029 = std::sin(v0025);
  const Scalar v0027 = params.l_1;
  const Scalar v0001 = _x(3, 0);
  const Scalar v0382 = -v0029;
  const Scalar v0297 = v0001 * v0027;
  const Scalar v0026 = std::cos(v0025);
  const Scalar v0000 = _x(2, 0);
  const Scalar v0328 = v0026 * v0026;
  const Scalar v0031 = v0000 + v0297 * v0382;
  const Scalar v0376 = v0297 * v0328;
  const Scalar v0033 = v0297 * v0376 + v0031 * v0031;
  const Scalar v0042 = std::sqrt(v0033);
  const Scalar v0315 = static_cast<Scalar>(1.5) * v0042;
  const Scalar v0306 = v0031 * static_cast<Scalar>(2);
  const Scalar v0305 = v0027 * static_cast<Scalar>(2);
  const Scalar v0233 = v0031 * v0382 + v0376;
  const bool v0035 = static_cast<Scalar>(0) < v0033;
  Scalar v0044;
  Scalar v0061;
  if (v0035) {
    v0044 = v0306 * v0315;
    v0061 = v0233 * v0305 * v0315;
  } else {
    v0044 = static_cast<Scalar>(0);
    v0061 = static_cast<Scalar>(0);
  }
  const Scalar v0008 = params.m_1;
  const Scalar v0019 = params.g;
  const Scalar v0063 = _f_mass(1, 0);
  const Scalar v0004 = _f_mass(0, 0);
  const Scalar v0009 = params.m_b + v0008;
  const Scalar v0016 = static_cast<Scalar>(1) / v0013;
  const Scalar v0299 = v0008 * v0029;
  const Scalar v0298 = (static_cast<Scalar>(-0.16666666666666666)) * params.c_d_1;
  const Scalar v0380 = (v0009 * params.mu_b) * -v0019;
  const Scalar v0018 = std::tanh(v0000 * v0016);
  const Scalar v0066 = static_cast<Scalar>(1) / v0027;
  const Scalar v0003 = _f_base(0, 0);
  const Scalar v0002 = u;
  const Scalar v0324 = v0008 * (v0009 + v0299 * v0382);
  const Scalar v0312 = v0009 * (static_cast<Scalar>(1) / (v0027 * v0027));
  const Scalar v0243 = v0061 * v0298 + v0027 * (v0004 * v0382 + v0026 * (v0063 + v0019 * -v0008));
  const Scalar v0372 = v0066 * v0299;
  const Scalar v0048 =
      v0002 + v0003 + v0004 + v0018 * v0380 + v0044 * v0298 + v0008 * v0297 * (v0001 * v0026);
  const Scalar v0301 = v0029 * v0066;
  const Scalar v0052 = static_cast<Scalar>(1) / v0324;
  const Scalar v0371 = v0052 * v0066;
  const Scalar v0182 = v0048 * v0372 + v0243 * v0312;
  const Scalar v0178 = v0048 + v0243 * v0301;
  const Scalar v0304 = v0008 * v0052;
  if (static_cast<bool>(_J_x)) {
    const Scalar v0386 = -v0026;
    Scalar v0095;
    Scalar v0108;
    Scalar v0117;
    Scalar v0126;
    Scalar v0138;
    if (v0035) {
      const Scalar v0087 = static_cast<Scalar>(1) / v0042;
      const Scalar v0082 = static_cast<Scalar>(0.75);
      const Scalar v0307 = static_cast<Scalar>(2) * v0082;
      const Scalar v0374 = v0307 * (v0087 * v0233);
      const Scalar v0245 = v0031 + v0029 * v0297;
      const Scalar v0367 = v0042 * static_cast<Scalar>(3);
      v0095 = -((v0297 * v0374 + v0315) * (v0026 * v0027) * (static_cast<Scalar>(2) * v0245));
      v0108 = (v0087 * v0245 * (v0306 * v0307) + v0367) * v0297 * v0386;
      v0117 = v0027 * (v0367 * v0382 + v0306 * v0374);
      v0126 = v0306 * v0306 * (v0082 * v0087) + v0367;
      v0138 = v0027 * (v0233 * v0374 + (v0328 + v0029 * v0029) * v0315) * v0305;
    } else {
      v0095 = static_cast<Scalar>(0);
      v0108 = static_cast<Scalar>(0);
      v0117 = static_cast<Scalar>(0);
      v0126 = static_cast<Scalar>(0);
      v0138 = static_cast<Scalar>(0);
    }
    const Scalar v0259 = v0095 * v0298 + v0027 * (v0004 * v0386 + v0029 * (v0008 * v0019 + -v0063));
    const Scalar v0110 = v0297 * v0299 * -v0001 + v0108 * v0298;
    const Scalar v0310 = v0008 * v0026;
    const Scalar v0373 = v0117 * v0298;
    const Scalar v0077 = static_cast<Scalar>(1) / (v0324 * v0324);
    const Scalar v0142 = static_cast<Scalar>(2) * v0297 * v0310 + v0373;
    const Scalar v0375 = v0138 * v0298;
    const Scalar v0128 = v0016 * (static_cast<Scalar>(1) + v0018 * -v0018) * v0380 + v0126 * v0298;
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(1);
    _J_x(0, 3) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(1);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = v0008 * (v0178 * v0299 * v0310 * (static_cast<Scalar>(2) * v0077) +
                          v0052 * (v0110 + v0066 * (v0029 * v0259 + v0026 * v0243)));
    _J_x(2, 2) = (v0128 + v0301 * v0373) * v0304;
    _J_x(2, 3) = (v0142 + v0301 * v0375) * v0304;
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v0052 * v0259 * v0312 + v0008 * (v0077 * v0182 * v0299 * (v0026 * static_cast<Scalar>(2)) +
                                         (v0026 * v0048 + v0029 * v0110) * v0371);
    _J_x(3, 2) = v0052 * (v0312 * v0373 + v0128 * v0372);
    _J_x(3, 3) = v0052 * (v0312 * v0375 + v0142 * v0372);
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = v0304;
    _J_u(3, 0) = v0299 * v0371;
  }
  _x_dot(0, 0) = v0000;
  _x_dot(1, 0) = v0001;
  _x_dot(2, 0) = v0178 * v0304;
  _x_dot(3, 0) = v0052 * v0182;
}

}  // namespace gen

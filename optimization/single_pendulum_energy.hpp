// Machine generated code.
#pragma once
#include <cmath>
#include <cstdint>

#include <wrenfold/span.h>

// User-specified imports:
#include "parameters.hpp"

namespace gen {

template <typename Scalar, typename T1, typename T4, typename T5>
void single_pendulum_energy(const pendulum::PendulumParams& params, const T1& x, Scalar& kinetic,
                            Scalar& potential, T4&& kin_D_x, T5&& pot_D_x) {
  auto _x = wf::make_input_span<4, 1>(x);
  auto _kin_D_x = wf::make_optional_output_span<1, 4>(kin_D_x);
  auto _pot_D_x = wf::make_optional_output_span<1, 4>(pot_D_x);

  // Operation counts:
  // add: 6
  // branch: 2
  // call: 2
  // multiply: 19
  // negate: 2
  // total: 31

  const Scalar v001 = _x(2, 0);
  const Scalar v007 = _x(1, 0);
  const Scalar v009 = params.l_1;
  const Scalar v006 = _x(3, 0);
  const Scalar v064 = -v001;
  const Scalar v008 = std::sin(v007);
  const Scalar v015 = params.m_1;
  const Scalar v049 = v006 * v009;
  const Scalar v040 = v008 * v064;
  const Scalar v024 = std::cos(v007);
  const Scalar v050 = v009 * v015;
  const Scalar v053 = v001 * params.m_b;
  if (static_cast<bool>(_kin_D_x)) {
    _kin_D_x(0, 0) = static_cast<Scalar>(0);
    _kin_D_x(0, 1) = v049 * (v015 * v024) * v064;
    _kin_D_x(0, 2) = v015 * (v001 + v049 * -v008) + v053;
    _kin_D_x(0, 3) = (v040 + v049) * v050;
  }
  const Scalar v063 = params.g * v050;
  if (static_cast<bool>(_pot_D_x)) {
    _pot_D_x(0, 0) = static_cast<Scalar>(0);
    _pot_D_x(0, 1) = v024 * v063;
    _pot_D_x(0, 2) = static_cast<Scalar>(0);
    _pot_D_x(0, 3) = static_cast<Scalar>(0);
  }
  const Scalar v000 = static_cast<Scalar>(0.5);
  const Scalar v051 = v000 * v001;
  kinetic = v051 * v053 + v015 * (v001 * v051 + (v040 + v000 * v049) * v049);
  potential = v008 * v063;
}

}  // namespace gen

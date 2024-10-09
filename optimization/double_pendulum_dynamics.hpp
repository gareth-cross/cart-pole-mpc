// Machine generated code.
#pragma once
#include <cmath>
#include <cstdint>

#include <wrenfold/span.h>

// User-specified imports:
#include "structs.hpp"

namespace gen {

template <typename Scalar, typename T1, typename T3, typename T4, typename T5>
void double_pendulum_dynamics(const pendulum::DoubleCartPoleParams& params, const T1& x,
                              const Scalar u, T3&& x_dot, T4&& J_x, T5&& J_u) {
  auto _x = wf::make_input_span<6, 1>(x);
  auto _x_dot = wf::make_output_span<6, 1>(x_dot);
  auto _J_x = wf::make_optional_output_span<6, 6>(J_x);
  auto _J_u = wf::make_optional_output_span<6, 1>(J_u);

  // Operation counts:
  // add: 76
  // branch: 2
  // call: 4
  // divide: 2
  // multiply: 269
  // negate: 15
  // total: 368

  const Scalar v00012 = _x(2, 0);
  const Scalar v00004 = _x(1, 0);
  const Scalar v00020 = std::sin(v00012);
  const Scalar v00019 = std::sin(v00004);
  const Scalar v00013 = std::cos(v00012);
  const Scalar v00005 = std::cos(v00004);
  const Scalar v00010 = params.m_2;
  const Scalar v00977 = v00019 * v00020;
  const Scalar v01086 = params.m_1 + v00010;
  const Scalar v00249 = v00005 * v00013 + v00977;
  const Scalar v01089 = -v00010;
  const Scalar v01097 = -v01086;
  const Scalar v00038 = params.m_b + v01086;
  const Scalar v01090 = -v00019;
  const Scalar v00508 = v00019 * v01097;
  const Scalar v00002 = _x(5, 0);
  const Scalar v00014 = params.l_2;
  const Scalar v00001 = _x(4, 0);
  const Scalar v00008 = params.l_1;
  const Scalar v00512 = v00038 + v00020 * v00020 * v01089;
  const Scalar v01033 = v01090 * v01097;
  const Scalar v00913 = v00038 * v00249;
  const Scalar v00941 = v00013 * v00019;
  const Scalar v00869 = v00002 * v00002;
  const Scalar v00842 = v00010 * v00014;
  const Scalar v00923 = v00005 * v00020;
  const Scalar v00865 = v00001 * v00001;
  const Scalar v00847 = v00008 * v01086;
  const Scalar v00853 = v00001 * v00008;
  const Scalar v00358 = v00020 * -v00005 + v00941;
  const Scalar v00051 = params.g;
  const Scalar v01020 = v00005 * v01097;
  const Scalar v01015 = v00842 * v00869;
  const Scalar v00309 = v00013 * v01090 + v00923;
  const Scalar v01017 = v00847 * v00865;
  const Scalar v00843 = v00008 * v00014;
  const Scalar v00694 = v00249 * v00913 * v01089 +
                        ((v00010 * static_cast<Scalar>(2)) * (v00020 * v00249) + v00508) * v01033 +
                        v00512 * v01086;
  const Scalar v00686 = v00020 + v00249 * v01090;
  const Scalar v01087 = -v00008;
  const Scalar v00521 = v00508 + v00020 * (v00010 * v00249);
  const Scalar v00003 = u;
  const Scalar v01006 = v00842 * v00843;
  const Scalar v00864 = v00686 * v01086;
  const Scalar v00844 = v00008 * v00010;
  const Scalar v00535 = v00051 * -v00013 + v00001 * v00358 * v00853;
  const Scalar v00529 = v00309 * v01015 + v00051 * v01020;
  const Scalar v00543 = v00977 * v01097 + v00913;
  const Scalar v01098 = -v00521;
  const Scalar v00246 = v00003 + v00013 * v01015 + v00005 * v01017;
  const Scalar v00689 = v00038 + v01033 * v01090;
  const Scalar v00683 = (v00249 * v00249) * v01089 + v01086;
  const Scalar v00045 = static_cast<Scalar>(1) / ((v00008 * v00694) * v01006);
  const Scalar v01022 = v00689 * v00847;
  const Scalar v01021 = v00543 * v01087;
  const Scalar v01011 = v00686 * v00847;
  const Scalar v01010 = v00683 * v00843;
  const Scalar v00878 = v00008 * v00246;
  const Scalar v00858 = v00008 * v00045;
  const Scalar v01077 = (v00246 * v01098 + v00535 * v00543 * v01089 + v00512 * v00529) * v01006;
  const Scalar v01013 = v00842 * v00858;
  const Scalar v00710 = v00246 * v01011 + v00529 * v01021 + v00535 * v01022;
  const Scalar v00705 =
      (v00529 * (v00521 * v01087) + v00535 * v00844 * v00864) * v00843 + v00878 * v01010;
  const Scalar v01009 = v00045 * v00842;
  if (static_cast<bool>(_J_x)) {
    const Scalar v01038 = v00019 * v01087;
    const Scalar v00600 = v00020 * v00358 + v00013 * v00249;
    const Scalar v01080 = v00694 * v01006;
    const Scalar v00849 = v00008 * v00008;
    const Scalar v00992 = v00038 * v00358;
    const Scalar v00867 = v00008 * v00249;
    const Scalar v00861 = v00019 * v01086;
    const Scalar v00719 = v00005 * v00249 + v00019 * v00309;
    const Scalar v00105 = static_cast<Scalar>(1) / (v00849 * v01080 * v01080);
    const Scalar v01032 = v00865 * v01087;
    const Scalar v01057 = v00719 * v00844;
    const Scalar v00756 = v00867 * v00992 + (v00013 * (v00008 * v00020) + v00600 * v01038) * v01086;
    const Scalar v01047 = v00521 * -v00014;
    const Scalar v00572 = v01015 * -v00249 + v00051 * v00861;
    const Scalar v00880 = v00008 * v00529;
    const Scalar v00564 = v00309 * (v00010 * v00020) + v01020;
    const Scalar v00623 = v00923 * v01097 + v00038 * v00309;
    const Scalar v01065 = v00756 * (static_cast<Scalar>(2) * v00105);
    const Scalar v01008 = v00842 * v01087;
    const Scalar v00634 = v00941 * v01097 + v00992;
    const Scalar v00855 = v00008 * static_cast<Scalar>(2);
    const Scalar v00604 = v00020 * v00051 + v00249 * v01032;
    const Scalar v00744 = v00013 + v00358 * v01090;
    const Scalar v01096 = -v00843;
    const Scalar v01037 = v00008 * v00842;
    const Scalar v01012 = v00842 * v00847;
    const Scalar v00890 = static_cast<Scalar>(2) * v00045;
    const Scalar v01007 = v00842 * v00842;
    const Scalar v00728 =
        (v00005 * (v00861 * v01087) + v00020 * v01057) * v01097 + v00309 * v00844 * v00913;
    const Scalar v00866 = v00005 * v01086;
    const Scalar v01091 = -static_cast<Scalar>(2);
    const Scalar v01100 = -v00105;
    const Scalar v01083 = v01007 * v01008;
    const Scalar v01078 = v01007 * (v00855 * (v00002 * v00045));
    const Scalar v01085 = (v00001 * v00842) * (v00849 * v00890);
    _J_x(0, 0) = static_cast<Scalar>(0);
    _J_x(0, 1) = static_cast<Scalar>(0);
    _J_x(0, 2) = static_cast<Scalar>(0);
    _J_x(0, 3) = static_cast<Scalar>(1);
    _J_x(0, 4) = static_cast<Scalar>(0);
    _J_x(0, 5) = static_cast<Scalar>(0);
    _J_x(1, 0) = static_cast<Scalar>(0);
    _J_x(1, 1) = static_cast<Scalar>(0);
    _J_x(1, 2) = static_cast<Scalar>(0);
    _J_x(1, 3) = static_cast<Scalar>(0);
    _J_x(1, 4) = static_cast<Scalar>(1);
    _J_x(1, 5) = static_cast<Scalar>(0);
    _J_x(2, 0) = static_cast<Scalar>(0);
    _J_x(2, 1) = static_cast<Scalar>(0);
    _J_x(2, 2) = static_cast<Scalar>(0);
    _J_x(2, 3) = static_cast<Scalar>(0);
    _J_x(2, 4) = static_cast<Scalar>(0);
    _J_x(2, 5) = static_cast<Scalar>(1);
    _J_x(3, 0) = static_cast<Scalar>(0);
    _J_x(3, 1) =
        v00705 * v01006 * (v00842 * v01091) * (v00728 * v01100) +
        (((v00564 * v00880 + v00572 * (v00008 * v00521) + (v00535 * v01086) * v01057) * v01096 +
          ((static_cast<Scalar>(2) * v00246) * (v00309 * v01087) +
           v00847 * v00853 * (v00001 * v00686)) *
              v00842 * v00867) +
         v01010 * v01017 * v01038) *
            v01009;
    _J_x(3, 2) = (v00105 * v00756) * (v00705 * v01091) * v01083 +
                 (v00604 * (v00008 * v00686) * v01012 + v00744 * (v00008 * v00535) * v01012 +
                  (v00246 * v00855 * (v00249 * v00358) + v00020 * v00869 * v01010 +
                   v00600 * v00880 + v00249 * v00843 * (v00521 * v00869)) *
                      v01008) *
                     v01009;
    _J_x(3, 3) = static_cast<Scalar>(0);
    _J_x(3, 4) = (v00358 * v00842 * v01011 + v00866 * v01010) * v01085;
    _J_x(3, 5) = (v00013 * v01010 + v00309 * (v00843 * v01098)) * v01078;
    _J_x(4, 0) = static_cast<Scalar>(0);
    _J_x(4, 1) = v01006 * (v00728 * v01091) * v01077 * v01100 +
                 (v00512 * v00572 * v00843 +
                  (v00246 * v00564 + v00623 * (v00010 * v00535) + v00521 * v00861 * v01032 +
                   v00844 * v00865 * (v00249 * v00543)) *
                      v01096) *
                     v01009;
    _J_x(4, 2) =
        ((v00600 * -v00246 + v00634 * -v00535 + v00512 * (v00002 * v00014) * (v00002 * v00249)) *
             v01037 +
         v00543 * v00604 * v01008) *
            v01009 +
        -((v00020 * (v00013 * static_cast<Scalar>(2) * v00529 + v00869 * v01047) * v01013 +
           v01077 * (v01008 * v01065)) *
          v00842);
    _J_x(4, 3) = static_cast<Scalar>(0);
    _J_x(4, 4) =
        (v00358 * (v00010 * v00543) + v00521 * v00866) * v01006 * (v00001 * v00890) * v01087;
    _J_x(4, 5) = (v00013 * v01047 + v00512 * (v00014 * v00309)) * v01078;
    _J_x(5, 0) = static_cast<Scalar>(0);
    _J_x(5, 1) =
        v00710 * (v00105 * v00728) * (v01006 * v01008) * v01091 +
        v00045 *
            ((v00623 * v00880 + v00572 * (v00008 * v00543) + v00019 * v01011 * v01017) * v01008 +
             (v00008 *
                  (static_cast<Scalar>(2) * v00535 * v01020 * v01033 + v00249 * v00689 * v01017) +
              v00719 * (v00246 * v01086) * v01087) *
                 v01037);
    _J_x(5, 2) = (v00710 * v01087) * v01065 * v01083 +
                 (v00878 * (v00744 * v00847) + v00634 * v00880 * v01087 +
                  (v00008 * v00689) * (v00604 * v00847) +
                  (v00020 * v01011 + v00543 * v00867) * v00869 * v01008) *
                     v01009;
    _J_x(5, 3) = static_cast<Scalar>(0);
    _J_x(5, 4) = (v00358 * v01022 + v00864 * (v00005 * v00847)) * v01085;
    _J_x(5, 5) = (v00013 * v01011 + v00309 * v01021) * v01078;
  }
  if (static_cast<bool>(_J_u)) {
    _J_u(0, 0) = static_cast<Scalar>(0);
    _J_u(1, 0) = static_cast<Scalar>(0);
    _J_u(2, 0) = static_cast<Scalar>(0);
    _J_u(3, 0) = v00683 * v00858 * v01006;
    _J_u(4, 0) = (v00045 * v01006) * v01098;
    _J_u(5, 0) = v01011 * v01013;
  }
  const Scalar v00000 = _x(3, 0);
  _x_dot(0, 0) = v00000;
  _x_dot(1, 0) = v00001;
  _x_dot(2, 0) = v00002;
  _x_dot(3, 0) = v00705 * v01009;
  _x_dot(4, 0) = v00045 * v01077;
  _x_dot(5, 0) = v00710 * v01013;
}

}  // namespace gen

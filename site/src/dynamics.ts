// Machine generated code (see generate.py)
import * as mathjs from 'mathjs';
import { PendulumParams } from './pendulum_params';

export function pendulum_dynamics(
  params: PendulumParams,
  x: mathjs.Matrix,
  u: number,
  compute_J_x: boolean,
  compute_J_u: boolean
): { x_dot: mathjs.Matrix; J_x: mathjs.Matrix; J_u: mathjs.Matrix } {
  // Check input dimensions:
  if (x.size()[0] !== 6 || x.size()[1] !== 1) {
    throw new Error(`Wrong shape for x: ${x.size()}`);
  }
  // Declare output arrays:
  const x_dot = mathjs.matrix([]);
  const J_x = mathjs.matrix([]);
  const J_u = mathjs.matrix([]);
  // Operation counts:
  // add: 76
  // branch: 2
  // call: 4
  // divide: 2
  // multiply: 269
  // negate: 15
  // total: 368
  const v00012 = x.get([2, 0]);
  const v00004 = x.get([1, 0]);
  const v00020 = mathjs.sin(v00012);
  const v00019 = mathjs.sin(v00004);
  const v00013 = mathjs.cos(v00012);
  const v00005 = mathjs.cos(v00004);
  const v00010 = params.m_2;
  const v00977 = v00019 * v00020;
  const v01086 = params.m_1 + v00010;
  const v00249 = v00005 * v00013 + v00977;
  const v01089 = -v00010;
  const v01097 = -v01086;
  const v00038 = params.m_b + v01086;
  const v01090 = -v00019;
  const v00508 = v00019 * v01097;
  const v00002 = x.get([5, 0]);
  const v00014 = params.l_2;
  const v00001 = x.get([4, 0]);
  const v00008 = params.l_1;
  const v00512 = v00038 + v00020 * v00020 * v01089;
  const v01033 = v01090 * v01097;
  const v00913 = v00038 * v00249;
  const v00941 = v00013 * v00019;
  const v00869 = v00002 * v00002;
  const v00842 = v00010 * v00014;
  const v00923 = v00005 * v00020;
  const v00865 = v00001 * v00001;
  const v00847 = v00008 * v01086;
  const v00853 = v00001 * v00008;
  const v00358 = v00020 * -v00005 + v00941;
  const v00051 = params.g;
  const v01020 = v00005 * v01097;
  const v01015 = v00842 * v00869;
  const v00309 = v00013 * v01090 + v00923;
  const v01017 = v00847 * v00865;
  const v00843 = v00008 * v00014;
  const v00694 =
    v00249 * v00913 * v01089 +
    (v00010 * Number(2) * (v00020 * v00249) + v00508) * v01033 +
    v00512 * v01086;
  const v00686 = v00020 + v00249 * v01090;
  const v01087 = -v00008;
  const v00521 = v00508 + v00020 * (v00010 * v00249);
  const v00003 = u;
  const v01006 = v00842 * v00843;
  const v00864 = v00686 * v01086;
  const v00844 = v00008 * v00010;
  const v00535 = v00051 * -v00013 + v00001 * v00358 * v00853;
  const v00529 = v00309 * v01015 + v00051 * v01020;
  const v00543 = v00977 * v01097 + v00913;
  const v01098 = -v00521;
  const v00246 = v00003 + v00013 * v01015 + v00005 * v01017;
  const v00689 = v00038 + v01033 * v01090;
  const v00683 = v00249 * v00249 * v01089 + v01086;
  const v00045 = Number(1) / (v00008 * v00694 * v01006);
  const v01022 = v00689 * v00847;
  const v01021 = v00543 * v01087;
  const v01011 = v00686 * v00847;
  const v01010 = v00683 * v00843;
  const v00878 = v00008 * v00246;
  const v00858 = v00008 * v00045;
  const v01077 =
    (v00246 * v01098 + v00535 * v00543 * v01089 + v00512 * v00529) * v01006;
  const v01013 = v00842 * v00858;
  const v00710 = v00246 * v01011 + v00529 * v01021 + v00535 * v01022;
  const v00705 =
    (v00529 * (v00521 * v01087) + v00535 * v00844 * v00864) * v00843 +
    v00878 * v01010;
  const v01009 = v00045 * v00842;
  if (compute_J_x) {
    const v01038 = v00019 * v01087;
    const v00600 = v00020 * v00358 + v00013 * v00249;
    const v01080 = v00694 * v01006;
    const v00849 = v00008 * v00008;
    const v00992 = v00038 * v00358;
    const v00867 = v00008 * v00249;
    const v00861 = v00019 * v01086;
    const v00719 = v00005 * v00249 + v00019 * v00309;
    const v00105 = Number(1) / (v00849 * v01080 * v01080);
    const v01032 = v00865 * v01087;
    const v01057 = v00719 * v00844;
    const v00756 =
      v00867 * v00992 + (v00013 * (v00008 * v00020) + v00600 * v01038) * v01086;
    const v01047 = v00521 * -v00014;
    const v00572 = v01015 * -v00249 + v00051 * v00861;
    const v00880 = v00008 * v00529;
    const v00564 = v00309 * (v00010 * v00020) + v01020;
    const v00623 = v00923 * v01097 + v00038 * v00309;
    const v01065 = v00756 * (Number(2) * v00105);
    const v01008 = v00842 * v01087;
    const v00634 = v00941 * v01097 + v00992;
    const v00855 = v00008 * Number(2);
    const v00604 = v00020 * v00051 + v00249 * v01032;
    const v00744 = v00013 + v00358 * v01090;
    const v01096 = -v00843;
    const v01037 = v00008 * v00842;
    const v01012 = v00842 * v00847;
    const v00890 = Number(2) * v00045;
    const v01007 = v00842 * v00842;
    const v00728 =
      (v00005 * (v00861 * v01087) + v00020 * v01057) * v01097 +
      v00309 * v00844 * v00913;
    const v00866 = v00005 * v01086;
    const v01091 = -Number(2);
    const v01100 = -v00105;
    const v01083 = v01007 * v01008;
    const v01078 = v01007 * (v00855 * (v00002 * v00045));
    const v01085 = v00001 * v00842 * (v00849 * v00890);
    J_x.resize([6, 6]);
    J_x.set([0, 0], Number(0));
    J_x.set([0, 1], Number(0));
    J_x.set([0, 2], Number(0));
    J_x.set([0, 3], Number(1));
    J_x.set([0, 4], Number(0));
    J_x.set([0, 5], Number(0));
    J_x.set([1, 0], Number(0));
    J_x.set([1, 1], Number(0));
    J_x.set([1, 2], Number(0));
    J_x.set([1, 3], Number(0));
    J_x.set([1, 4], Number(1));
    J_x.set([1, 5], Number(0));
    J_x.set([2, 0], Number(0));
    J_x.set([2, 1], Number(0));
    J_x.set([2, 2], Number(0));
    J_x.set([2, 3], Number(0));
    J_x.set([2, 4], Number(0));
    J_x.set([2, 5], Number(1));
    J_x.set([3, 0], Number(0));
    J_x.set(
      [3, 1],
      v00705 * v01006 * (v00842 * v01091) * (v00728 * v01100) +
        ((v00564 * v00880 +
          v00572 * (v00008 * v00521) +
          v00535 * v01086 * v01057) *
          v01096 +
          (Number(2) * v00246 * (v00309 * v01087) +
            v00847 * v00853 * (v00001 * v00686)) *
            v00842 *
            v00867 +
          v01010 * v01017 * v01038) *
          v01009
    );
    J_x.set(
      [3, 2],
      v00105 * v00756 * (v00705 * v01091) * v01083 +
        (v00604 * (v00008 * v00686) * v01012 +
          v00744 * (v00008 * v00535) * v01012 +
          (v00246 * v00855 * (v00249 * v00358) +
            v00020 * v00869 * v01010 +
            v00600 * v00880 +
            v00249 * v00843 * (v00521 * v00869)) *
            v01008) *
          v01009
    );
    J_x.set([3, 3], Number(0));
    J_x.set([3, 4], (v00358 * v00842 * v01011 + v00866 * v01010) * v01085);
    J_x.set([3, 5], (v00013 * v01010 + v00309 * (v00843 * v01098)) * v01078);
    J_x.set([4, 0], Number(0));
    J_x.set(
      [4, 1],
      v01006 * (v00728 * v01091) * v01077 * v01100 +
        (v00512 * v00572 * v00843 +
          (v00246 * v00564 +
            v00623 * (v00010 * v00535) +
            v00521 * v00861 * v01032 +
            v00844 * v00865 * (v00249 * v00543)) *
            v01096) *
          v01009
    );
    J_x.set(
      [4, 2],
      ((v00600 * -v00246 +
        v00634 * -v00535 +
        v00512 * (v00002 * v00014) * (v00002 * v00249)) *
        v01037 +
        v00543 * v00604 * v01008) *
        v01009 +
        -(
          (v00020 * (v00013 * Number(2) * v00529 + v00869 * v01047) * v01013 +
            v01077 * (v01008 * v01065)) *
          v00842
        )
    );
    J_x.set([4, 3], Number(0));
    J_x.set(
      [4, 4],
      (v00358 * (v00010 * v00543) + v00521 * v00866) *
        v01006 *
        (v00001 * v00890) *
        v01087
    );
    J_x.set([4, 5], (v00013 * v01047 + v00512 * (v00014 * v00309)) * v01078);
    J_x.set([5, 0], Number(0));
    J_x.set(
      [5, 1],
      v00710 * (v00105 * v00728) * (v01006 * v01008) * v01091 +
        v00045 *
          ((v00623 * v00880 +
            v00572 * (v00008 * v00543) +
            v00019 * v01011 * v01017) *
            v01008 +
            (v00008 *
              (Number(2) * v00535 * v01020 * v01033 +
                v00249 * v00689 * v01017) +
              v00719 * (v00246 * v01086) * v01087) *
              v01037)
    );
    J_x.set(
      [5, 2],
      v00710 * v01087 * v01065 * v01083 +
        (v00878 * (v00744 * v00847) +
          v00634 * v00880 * v01087 +
          v00008 * v00689 * (v00604 * v00847) +
          (v00020 * v01011 + v00543 * v00867) * v00869 * v01008) *
          v01009
    );
    J_x.set([5, 3], Number(0));
    J_x.set([5, 4], (v00358 * v01022 + v00864 * (v00005 * v00847)) * v01085);
    J_x.set([5, 5], (v00013 * v01011 + v00309 * v01021) * v01078);
  }
  if (compute_J_u) {
    J_u.resize([6, 1]);
    J_u.set([0, 0], Number(0));
    J_u.set([1, 0], Number(0));
    J_u.set([2, 0], Number(0));
    J_u.set([3, 0], v00683 * v00858 * v01006);
    J_u.set([4, 0], v00045 * v01006 * v01098);
    J_u.set([5, 0], v01011 * v01013);
  }
  const v00000 = x.get([3, 0]);
  x_dot.resize([6, 1]);
  x_dot.set([0, 0], v00000);
  x_dot.set([1, 0], v00001);
  x_dot.set([2, 0], v00002);
  x_dot.set([3, 0], v00705 * v01009);
  x_dot.set([4, 0], v00045 * v01077);
  x_dot.set([5, 0], v00710 * v01013);
  return { x_dot: x_dot, J_x: J_x, J_u: J_u };
}

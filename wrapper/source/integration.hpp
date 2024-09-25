// Copyright 2024 Gareth Cross.
#pragma once
#include <tuple>

#include <Eigen/Dense>

namespace pendulum {

template <int D, typename F>
auto runge_kutta_4th_order(const Eigen::Matrix<double, D, 1>& x, const double u, const double h,
                           F&& f) {
  using X = Eigen::Matrix<double, D, 1>;
  using XJacobian = Eigen::Matrix<double, D, D>;
  using UJacobian = Eigen::Matrix<double, D, 1>;

  // Storage for jacobians of the integration function:
  XJacobian k1_D_x;
  XJacobian k2_D_x_arg;
  XJacobian k3_D_x_arg;
  XJacobian k4_D_x_arg;
  UJacobian k1_D_u;
  UJacobian k2_D_u_arg;
  UJacobian k3_D_u_arg;
  UJacobian k4_D_u_arg;

  const X k1 = f(x, u, k1_D_x, k1_D_u);
  const X k2 = f(x + k1 * h / 2.0, u, k2_D_x_arg, k2_D_u_arg);
  const X k3 = f(x + k2 * h / 2.0, u, k3_D_x_arg, k3_D_u_arg);
  const X k4 = f(x + k3 * h, u, k4_D_x_arg, k4_D_u_arg);
  const X result = x + (h / 6.0) * (k1 + k2 * 2.0 + k3 * 2.0 + k4);

  const auto I = XJacobian::Identity();
  const XJacobian k2_D_x = k2_D_x_arg * (I + k1_D_x * h / 2.0);
  const XJacobian k3_D_x = k3_D_x_arg * (I + k2_D_x * h / 2.0);
  const XJacobian k4_D_x = k4_D_x_arg * (I + k3_D_x * h);

  const UJacobian k2_D_u = k2_D_x_arg * k1_D_u * (h / 2.0) + k2_D_u_arg;
  const UJacobian k3_D_u = k3_D_x_arg * k2_D_u * (h / 2.0) + k3_D_u_arg;
  const UJacobian k4_D_u = k4_D_x_arg * k3_D_u * h + k4_D_u_arg;

  const XJacobian x_new_D_x = I + (h / 6.0) * (k1_D_x + k2_D_x * 2.0 + k3_D_x * 2.0 + k4_D_x);
  const UJacobian x_new_D_u = (h / 6.0) * (k1_D_u + k2_D_u * 2.0 + k3_D_u * 2.0 + k4_D_u);

  return std::make_tuple(result, x_new_D_x, x_new_D_u);
}

}  // namespace pendulum

// Copyright 2024 Gareth Cross.
#include "simulator.hpp"

#include "mini_opt/assertions.hpp"

#include "integration.hpp"
#include "single_pendulum_dynamics.hpp"

namespace pendulum {

void Simulator::Step(double dt, const double u) {
  F_ASSERT_GE(dt, 0.0);
  F_ASSERT(std::isfinite(u), "u = {}", u);

  // The caller may be running at some variable rate, so we break
  // the interval into fixed sub-steps.
  constexpr double internal_dt = 0.001;
  while (dt > 0.0) {
    SubStep(std::min(dt, internal_dt), u);
    dt -= internal_dt;
  }
}

void Simulator::SubStep(const double dt, const double u) {
  state_ = runge_kutta_4th_order_no_jacobians<4>(
      state_, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u_integration) {
        Eigen::Matrix<double, 4, 1> x_dot;
        gen::single_pendulum_dynamics(params_, x_updated, u_integration, x_dot, nullptr, nullptr);
        return x_dot;
      });
}

}  // namespace pendulum

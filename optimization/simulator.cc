// Copyright 2024 Gareth Cross.
#include "simulator.hpp"

#include "mini_opt/assertions.hpp"

#include "integration.hpp"
#include "single_pendulum_dynamics.hpp"

namespace pendulum {

void Simulator::Step(double dt, const double u, const Vector2& f_base, const Vector2& f_mass) {
  F_ASSERT_GE(dt, 0.0);
  F_ASSERT(std::isfinite(u), "u = {}", u);

  // The caller may be running at some variable rate, so we break
  // the interval into fixed sub-steps.
  constexpr double internal_dt = 0.001;
  while (dt > 0.0) {
    SubStep(std::min(dt, internal_dt), u, f_base, f_mass);
    dt -= internal_dt;
  }
}

void Simulator::SubStep(const double dt, const double u, const Vector2& f_base,
                        const Vector2& f_mass) {
  state_ = runge_kutta_4th_order_no_jacobians<4>(
      state_, u, dt, [&](const Eigen::Matrix<double, 4, 1>& x_updated, const double u_integration) {
        Eigen::Matrix<double, 4, 1> x_dot;
        gen::single_pendulum_dynamics(params_, x_updated, u_integration,
                                      Eigen::Vector2d(f_base.x, f_base.y),
                                      Eigen::Vector2d(f_mass.x, f_mass.y), x_dot, nullptr, nullptr);
        return x_dot;
      });
  state_[1] = mod_pi(state_[1]);
}

}  // namespace pendulum

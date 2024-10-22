// Copyright (c) 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

#include "structs.hpp"

namespace pendulum {

// Encapsulate the system state and provide interface to integrate it forward in time.
class Simulator {
 public:
  // Step the simulator forward by the specified amount of time.
  // `u` is the control input.
  void Step(const SingleCartPoleParams& params, double dt, double u, const Vector2& f_base,
            const Vector2& f_mass);

  // Get the state of the system.
  SingleCartPoleState GetState() const noexcept {
    return SingleCartPoleState{state_[0], state_[1], state_[2], state_[3]};
  }

  void SetState(const SingleCartPoleState& state) noexcept { state_ = state.ToVector(); }

 private:
  void SubStep(const SingleCartPoleParams& params, double dt, double u, const Vector2& f_base,
               const Vector2& f_mass);

  Eigen::Vector4d state_{0.0, -M_PI / 2, 0.0, 0.0};
};

}  // namespace pendulum

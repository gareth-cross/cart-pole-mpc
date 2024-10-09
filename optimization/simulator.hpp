// Copyright 2024 Gareth Cross.
#pragma once
#include <Eigen/Core>

#include "structs.hpp"

namespace pendulum {

// Encapsulate the system state and provide interface to integrate it forward in time.
class Simulator {
 public:
  // Construct with parameters of the system.
  explicit Simulator(const SingleCartPoleParams& params) noexcept : params_(params) {}

  // Step the simulator forward by the specified amount of time.
  // `u` is the control input.
  void Step(double dt, double u);

  // Get the state of the system.
  SingleCartPoleState GetState() const noexcept {
    return SingleCartPoleState{state_[0], state_[1], state_[2], state_[3]};
  }

  void SetState(const SingleCartPoleState& state) noexcept { state_ = state.ToVector(); }

 private:
  void SubStep(double dt, double u);

  SingleCartPoleParams params_;
  Eigen::Vector4d state_{Eigen::Vector4d::Zero()};
};

}  // namespace pendulum

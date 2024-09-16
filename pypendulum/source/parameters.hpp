// Copyright 2024 Gareth Cross.
#pragma once

namespace pendulum {

// Struct used to pass parameters back and forth from Python to the dynamics model.
struct PendulumParams {
  double m_b;
  double m_1;
  double m_2;
  double l_1;
  double l_2;
  double g;
};

}  // namespace pendulum

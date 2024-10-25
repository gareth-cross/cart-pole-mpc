// Copyright (c) 2024 Gareth Cross.

// See structs.hpp for docs.
export interface SingleCartPoleState {
  b_x: number;
  th_1: number;
  b_x_dot: number;
  th_1_dot: number;
}

// See structs.hpp for docs.
export interface SingleCartPoleParams {
  m_b: number;
  m_1: number;
  l_1: number;
  g: number;
  mu_b: number;
  v_mu_b: number;
  c_d_1: number;
  x_s: number;
  k_s: number;
}

export interface Point {
  x: number;
  y: number;
}

// See optimization.hpp for docs.
export interface OptimizationParams {
  control_dt: number;
  window_length: number;
  state_spacing: number;
  max_iterations: number;
  relative_exit_tol: number;
  absolute_first_derivative_tol: number;
  equality_penalty_initial: number;
  u_guess_sinusoid_amplitude: number;
  u_cost_weight: number;
  u_derivative_cost_weight: number;
  b_x_final_cost_weight: number;
  th_final_cost_weight: number;
  b_x_dot_final_cost_weight: number;
  th_dot_final_cost_weight: number;
}

import * as mathjs from 'mathjs';

import { pendulum_dynamics } from './dynamics';
import { PendulumParams } from './pendulum_params';

// Pendulum state as a struct so we can access named members.
export class PendulumState {
  // Base location on x-axis in meters.
  public b_x: number;
  // Angle of the first lever arm in radians.
  public theta_1: number;
  // Angle of the second lever arm in radians.
  public theta_2: number;
  // Linear velocity on x-axis in meters/second.
  public b_x_dot: number;
  // Angular rate of the first lever arm in rad/s.
  public theta_1_dot: number;
  // Angular rate of the second lever arm in rad/s.
  public theta_2_dot: number;

  constructor(
    b_x: number = 0,
    theta_1: number = 0,
    theta_2: number = 0,
    b_x_dot: number = 0,
    theta_1_dot: number = 0,
    theta_2_dot: number = 0
  ) {
    this.b_x = b_x;
    this.theta_1 = theta_1;
    this.theta_2 = theta_2;
    this.b_x_dot = b_x_dot;
    this.theta_1_dot = theta_1_dot;
    this.theta_2_dot = theta_2_dot;
  }

  // Convert from flat vector to structure.
  public static fromVector(x: mathjs.Matrix): PendulumState {
    const [rows, cols] = x.size();
    if (rows !== 6 || cols !== 1) {
      throw new Error(`Incorrect dimensions for x: [${rows}, ${cols}]`);
    }
    return new PendulumState(
      x.get([0, 0]),
      x.get([1, 0]),
      x.get([2, 0]),
      x.get([3, 0]),
      x.get([4, 0]),
      x.get([5, 0])
    );
  }
}

// Step the dynamical model of the pendulum.
export class Simulator {
  // Internal step size of the simulator.
  private sim_dt: number;

  // Physical parameters of the pendulum itself.
  private pendulum_params: PendulumParams;

  // The state of the system, ordered [b_x, theta_1, theta_2, b_x_dot, theta_1_dot, ...]
  private state: mathjs.Matrix;

  constructor(
    sim_dt: number,
    pendulum_params: PendulumParams,
    state: mathjs.Matrix
  ) {
    if (sim_dt <= 0.0) {
      throw new Error(`Simulator dt must be positive: ${sim_dt}`);
    }
    this.sim_dt = sim_dt;
    this.pendulum_params = pendulum_params;
    this.state = state;
  }

  // Step the simulation forward by `dt` seconds. `u` is the control input.
  public step(dt: number, u: number) {
    if (dt <= 0.0) {
      return;
    }

    // Break the simulation down into fixed interval sub-steps over which we integrate.
    let remaining = dt;
    while (remaining > 0.0) {
      this.sub_step(Math.min(remaining, this.sim_dt), u);
      remaining -= this.sim_dt;
    }
  }

  public getState(): PendulumState {
    return PendulumState.fromVector(this.state);
  }

  public getParams(): PendulumParams {
    return this.pendulum_params;
  }

  // Apply a single iteration of integration to the system.
  private sub_step(dt: number, u: number) {
    // f(x) computes the derivative at `x` with our current control input `u`:
    const f = (x: mathjs.Matrix) => {
      return pendulum_dynamics(this.pendulum_params, x, u, false, false).x_dot;
    };

    const k1 = f(this.state);
    const k2 = f(mathjs.add(this.state, mathjs.multiply(k1, dt / 2.0)));
    const k3 = f(mathjs.add(this.state, mathjs.multiply(k2, dt / 2.0)));
    const k4 = f(mathjs.add(this.state, mathjs.multiply(k3, dt)));

    // Runge-Kutta 4th order:
    //  f(n + 1) = f(n) + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    let df = mathjs.add(k1, mathjs.multiply(k2, 2.0));
    df = mathjs.add(df, mathjs.multiply(k3, 2.0));
    df = mathjs.add(df, k4);
    this.state = mathjs.add(this.state, mathjs.multiply(df, dt / 6.0));
  }
}

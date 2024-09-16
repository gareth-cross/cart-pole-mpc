import * as mathjs from 'mathjs';

import { Renderer } from './renderer';
import { Simulator } from './simulator';
import { PendulumParams } from './pendulum_params';

// TODO: temporary, initialize this somewhere else
const params = new PendulumParams(10.0, 0.1, 0.01, 0.5, 0.4, 9.81);
const x = mathjs.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).resize([6, 1]);

const simulator = new Simulator(0.001, params, x);
const renderer = new Renderer();

// The previous frame timestamp:
let previousTime: DOMHighResTimeStamp | null = null;

function animationCallback(timestamp: DOMHighResTimeStamp) {
  if (previousTime === null) {
    // First iteration, save time and schedule next frame.
    previousTime = timestamp;
    window.requestAnimationFrame(animationCallback);
    return;
  }

  const durationInSeconds = (timestamp - previousTime) / 1000.0;
  simulator.step(durationInSeconds, 0.0);
  renderer.draw(simulator.getState(), simulator.getParams());

  previousTime = timestamp;
  window.requestAnimationFrame(animationCallback);
}

// Start the animation loop:
window.requestAnimationFrame(animationCallback);

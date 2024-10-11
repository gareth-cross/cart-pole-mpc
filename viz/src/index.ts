import './styles.scss';
import 'bootstrap';

import plotlyJsDistMin from 'plotly.js-dist-min';

import OptimizationWasm, {
  MainModule,
  Simulator,
  SingleCartPoleParams,
  OptimizationParams,
  Optimization,
  SingleCartPoleState
} from './optimization-wasm';

import { Renderer } from './renderer';

function createPlot(id: string, title: string, xaxis: string, yaxis: string) {
  const layout = {
    title: title,
    xaxis: {
      title: xaxis
    },
    yaxis: {
      title: yaxis
    }
  };
  const newDiv = document.createElement('div');
  newDiv.id = id;
  plotlyJsDistMin.newPlot(newDiv, [], layout);
  document.getElementById('main-div').appendChild(newDiv);
}

function updatePlot(id: string, x: Array<number>, y: Array<number>) {
  const data = [
    {
      x: x,
      y: y,
      type: 'scatter' as const
    }
  ];
  plotlyJsDistMin.react(id, data);
}

class Application {
  private wasm: MainModule;
  private params: SingleCartPoleParams;
  private simulator: Simulator;
  private optimizer: Optimization;
  private renderer: Renderer;
  private previousTime: DOMHighResTimeStamp | null = null;
  private accumulatedTime: number = 0.0;
  private iteration: number = 0;
  private optimizationStepDurations: Array<number> = [];

  constructor(wasm: MainModule) {
    this.wasm = wasm;
    this.params = new this.wasm.SingleCartPoleParams(1.0, 0.1, 0.25, 9.81);
    this.simulator = new this.wasm.Simulator(this.params);
    const params = new this.wasm.OptimizationParams();
    this.optimizer = new this.wasm.Optimization(params);
    params.delete();
    this.renderer = new Renderer();
  }

  // Deallocate C++ resources.
  // This is used when running with `-fsanitize=address`.
  public cleanup() {
    this.params.delete();
    this.simulator.delete();
    this.optimizer.delete();
    this.params = null;
    this.simulator = null;
    this.optimizer = null;
  }

  public requestFrame() {
    window.requestAnimationFrame((ts) => this.animationCallback(ts));
  }

  private animationCallback(timestamp: DOMHighResTimeStamp) {
    if (this.previousTime === null) {
      // First iteration, save time and schedule next frame.
      this.previousTime = timestamp;
      this.requestFrame();
      return;
    }

    const durationInSeconds = (timestamp - this.previousTime) / 1000.0;
    const controlDt = 0.01;

    this.accumulatedTime += durationInSeconds;
    while (this.accumulatedTime >= controlDt) {
      this.stepControlAndSim(controlDt);
      this.accumulatedTime -= controlDt;
    }

    const current_state = this.simulator.getState();
    this.renderer.draw_single(current_state, this.params);
    current_state.delete();

    this.previousTime = timestamp;
    this.requestFrame();
    this.iteration++;
  }

  // Run the MPC and simulator.
  private stepControlAndSim(dt: number) {
    // Run the model predictive controller.
    const current_state = this.simulator.getState();
    const start_time = window.performance.now();
    const outputs = this.optimizer.step(current_state, this.params);
    const end_time = window.performance.now();

    // Collect samples for the optimization `step` duration.
    // Retain a sliding window of 1000.
    this.optimizationStepDurations.push((end_time - start_time) / 1000.0);
    if (this.optimizationStepDurations.length > 1000) {
      this.optimizationStepDurations.shift();
    }
    if (this.optimizationStepDurations.length > 1 && this.iteration % 200 == 0) {
      const averageDuration =
        this.optimizationStepDurations.reduce((a, b) => a + b) /
        this.optimizationStepDurations.length;
      const maxDuration = Math.max(...this.optimizationStepDurations);
      console.log(`Optimization times: mean = ${averageDuration}, max = ${maxDuration}`);
    }

    let times: Array<number> = [];
    for (let t = 0; t < outputs.windowLength(); ++t) {
      times.push(t * 0.01);
    }

    let u_controls: Array<number> = [];
    let x_states: Array<SingleCartPoleState> = [];
    times.forEach((_, index) => {
      u_controls.push(outputs.getControl(index));
      x_states.push(outputs.getPredictedState(index)); //  getPredictedState returns a weak ref.
    });

    const angles = x_states.map((x: SingleCartPoleState, _) => {
      return x.th_1;
    });

    // updatePlot('controlInputPlot', times, u_controls);
    // updatePlot('predictedAnglePlot', times, angles);

    this.simulator.step(dt, u_controls[0]);

    // We need to manually clean up C++ objects allocated via embind.
    outputs.delete();
    current_state.delete();
  }
}

document.addEventListener('DOMContentLoaded', () => {
  createPlot('controlInputPlot', 'Control Input', 'Time (s)', 'Force (N)');
  createPlot('predictedAnglePlot', 'Predicted Angle', 'Time (s)', 'Theta (rads)');

  OptimizationWasm().then((mod: MainModule) => {
    console.log('Loaded WASM module.');
    // Start the animation loop:
    const app = new Application(mod);
    app.requestFrame();
  });
});

import './styles.css';

// import plotlyJsDistMin from 'plotly.js-dist-min';
// import uPlot from 'uplot';

import OptimizationWasm, {
  MainModule,
  Simulator,
  SingleCartPoleParams,
  Optimization,
  SingleCartPoleState
} from './optimization-wasm';

import { Renderer } from './renderer';
import { Plotter } from './plotter';

// function createPlot(id: string, title: string, xaxis: string, yaxis: string) {
//   const plotCol = document.getElementById('plotColumn') as HTMLDivElement;
//   const layout = {
//     title: title,
//     xaxis: {
//       title: xaxis,
//       gridcolor: '#555'
//     },
//     yaxis: {
//       title: yaxis,
//       gridcolor: '#555'
//     },
//     paper_bgcolor: '#222',
//     plot_bgcolor: '#222',
//     font: {
//       color: '#fff'
//     },
//     margin: { l: 40, r: 5, t: 40, b: 40 }
//   };
//   const newDiv = document.createElement('div');
//   newDiv.id = id;
//   plotlyJsDistMin.newPlot(newDiv, [], layout);

//   new ResizeObserver(() => {
//     plotlyJsDistMin.update(
//       newDiv,
//       {},
//       { width: plotCol.getBoundingClientRect().width, height: 200.0 }
//     );
//   }).observe(plotCol);

//   plotCol.appendChild(newDiv);
// }

// function updatePlot(id: string, x: Array<number>, y: Array<number>) {
//   const data = [
//     {
//       x: x,
//       y: y,
//       type: 'scatter' as const
//     }
//   ];
//   const layout = {
//     // xaxis: {
//     //   autotick: false,
//     //   // ticks: 'outside',
//     //   tick0: 0,
//     //   dtick: 0.25,
//     //   ticklen: 8,
//     //   tickwidth: 4,
//     //   tickcolor: '#000'
//     // },
//     // yaxis: {
//     //   autotick: false,
//     //   // ticks: 'outside',
//     //   tick0: 0,
//     //   dtick: 0.25,
//     //   ticklen: 8,
//     //   tickwidth: 4,
//     //   tickcolor: '#000'
//     // }
//   };
//   plotlyJsDistMin.react(id, data, layout);
// }

class Application {
  private wasm: MainModule;
  private params: SingleCartPoleParams;
  private simulator: Simulator;
  private optimizer: Optimization;
  private renderer: Renderer;

  // Timing control:
  private previousTime: DOMHighResTimeStamp | null = null;
  private accumulatedTime: number = 0.0;

  // Plotters
  private controlPlotter: Plotter;

  // UI controls
  public controlEnabled: boolean = true;

  // For collecting execution times of the optimization:
  private optimizationStepDurations: Array<number> = [];
  private iteration: number = 0;

  constructor(wasm: MainModule) {
    this.wasm = wasm;
    this.params = new this.wasm.SingleCartPoleParams(1.0, 0.1, 0.25, 9.81);
    this.simulator = new this.wasm.Simulator(this.params);

    // Some params that we fix are configured up front:
    const params = new this.wasm.OptimizationParams();
    params.max_iterations = 8;

    this.optimizer = new this.wasm.Optimization(params);
    params.delete();

    this.renderer = new Renderer();

    this.controlPlotter = new Plotter('controlPlot', {
      yaxis_limit_lower: -30.0,
      yaxis_limit_upper: 30.0
    });
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
    if (durationInSeconds > 0.2) {
      // We may have been put in the background or paused. In this case, just set the previous
      // timestamp and request a new frame. We don't want to block while stepping through a long
      // time interval.
      this.previousTime = timestamp;
      this.requestFrame();
      return;
    }

    const controlDt = 0.01;
    this.accumulatedTime += durationInSeconds;
    while (this.accumulatedTime >= controlDt) {
      this.stepControlAndSim(controlDt);
      this.accumulatedTime -= controlDt;
    }

    const current_state = this.simulator.getState();
    this.renderer.drawSingle(current_state, this.params);
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

    let times: Float64Array = new Float64Array(outputs.windowLength());
    for (let t = 0; t < outputs.windowLength(); ++t) {
      times[t] = t * 0.01;
    }

    let u_controls: Float64Array = new Float64Array(outputs.windowLength());
    let x_states: Array<SingleCartPoleState> = [];
    times.forEach((_, index) => {
      u_controls[index] = outputs.getControl(index);
      x_states.push(outputs.getPredictedState(index)); //  getPredictedState returns a weak ref.
    });

    const angles = new Float64Array(
      x_states.map((x: SingleCartPoleState, _) => {
        return x.th_1;
      })
    );

    // updatePlot('controlInputPlot', times, u_controls);
    // updatePlot('predictedAnglePlot', times, angles);

    this.simulator.step(dt, this.controlEnabled ? u_controls[0] : 0.0);

    this.controlPlotter.data = { x: times, y: u_controls };
    this.controlPlotter.draw();

    // We need to manually clean up C++ objects allocated via embind.
    outputs.delete();
    current_state.delete();
  }
}

document.addEventListener('DOMContentLoaded', () => {
  // createPlot('controlInputPlot', 'Control Input', 'Time (s)', 'Force (N)');
  // createPlot('predictedAnglePlot', 'Predicted Angle', 'Time (s)', 'Theta (rads)');

  OptimizationWasm().then((mod: MainModule) => {
    console.log('Loaded WASM module.');
    const app = new Application(mod);

    // Connect buttons to the UI:
    document
      .getElementById('enableControllerCheckbox')
      .addEventListener('change', function (this: HTMLInputElement) {
        app.controlEnabled = this.checked;
      });

    // Start the animation loop:
    app.requestFrame();
  });
});

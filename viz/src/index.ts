// Copyright 2024 Gareth Cross.
import './styles.css';

import { saveAs } from 'file-saver';

import OptimizationWasm, {
  MainModule,
  Simulator,
  Optimization,
  OptimizationOutputs
} from './optimization-wasm';

import { Renderer } from './renderer';
import { Plotter } from './plotter';
import { TicToc } from './tic_toc';
import { MouseHandler, MouseInteraction } from './input';
import { Point, SingleCartPoleState, SingleCartPoleParams, OptimizationParams } from './interfaces';

class Application {
  private wasm: MainModule;
  private dynamicsParams: SingleCartPoleParams;
  private optimizationParams: OptimizationParams;
  private simulator: Simulator;
  private optimizer: Optimization;
  private renderer: Renderer;
  private mouseHandler: MouseHandler;

  // External forces placed by user interaction.
  // There is one element in this array per mass in the system.
  private externalForces: Array<Point>;

  // Timing control:
  private previousTime: DOMHighResTimeStamp | null = null;
  private accumulatedTime: number = 0.0;

  // Plotters
  private controlPlotter: Plotter;

  // An array of logged optimization outputs that we can dump to disk.
  private loggedMessages: Array<string> = [];

  // UI controls
  private controlEnabled: boolean = true;
  private simRate: number = 1.0;

  // For collecting execution times of the optimization:
  private optimizationTicToc: TicToc = new TicToc();
  private iteration: number = 0;

  constructor(wasm: MainModule) {
    this.wasm = wasm;
    this.dynamicsParams = {
      m_b: 1.0,
      m_1: 0.1,
      l_1: 0.25,
      g: 9.81,
      mu_b: 0.05,
      v_mu_b: 0.1,
      c_d_1: 0.02,
      x_s: 0.8,
      k_s: 100.0
    };
    this.simulator = new this.wasm.Simulator();

    // Some params that we fix are configured up front:
    this.optimizationParams = this.wasm.getDefaultOptimizationParams() as OptimizationParams;
    this.optimizer = new this.wasm.Optimization(this.optimizationParams);

    this.externalForces = [
      { x: 0, y: 0 },
      { x: 0, y: 0 }
    ];

    // The Renderer draws the cart-pole sim, and the plotters draw optimization outputs.
    this.renderer = new Renderer();
    this.mouseHandler = new MouseHandler();

    this.controlPlotter = new Plotter('controlPlot', {
      yAxisLimitLower: -50.0,
      yAxisLimitUpper: 50.0,
      gridX: {
        majorInterval: 0.1, //  Seconds.
        numMinorTicks: 0
      },
      gridY: {
        majorInterval: 10.0, //  Newtons.
        numMinorTicks: 0
      }
    });

    this.connectUi();
  }

  // Deallocate C++ resources.
  // This is used when running with `-fsanitize=address`.
  public cleanup() {
    this.simulator.delete();
    this.optimizer.delete();
    this.simulator = null;
    this.optimizer = null;
  }

  public requestFrame() {
    window.requestAnimationFrame((ts) => this.animationCallback(ts));
  }

  private connectCheckbox(inputId: string, callback: (value: boolean) => void) {
    const checkbox = document.getElementById(inputId) as HTMLInputElement;
    checkbox.addEventListener('change', () => {
      callback(checkbox.checked);
    });
  }

  // TODO: Could maybe use React for this, although that might be a bit heavy-handed.
  // For now I'll just do a bit of manual state management with addEventListener.
  private connectSliderElement(
    inputId: string,
    min: number,
    max: number,
    step: number,
    initialValue: number,
    setter: (value: number) => void,
    units?: string,
    behavior?: string
  ) {
    const slider = document.getElementById(inputId) as HTMLInputElement;
    const output = document.getElementById(inputId + 'Output');
    slider.min = min.toString();
    slider.max = max.toString();
    slider.step = step.toString();
    slider.value = initialValue.toString();
    output.innerHTML = initialValue.toFixed(2);
    if (units) {
      output.innerHTML += `${units}`;
    }
    slider.addEventListener(behavior || 'input', () => {
      const value = Math.min(Math.max(parseFloat(slider.value), min), max);
      setter(value);
      output.innerHTML = value.toFixed(2) + (units ? `${units}` : '');
    });
  }

  private connectUi() {
    this.connectCheckbox('enableControllerCheckbox', (value) => {
      if (!this.controlEnabled && value) {
        this.optimizer.reset();
      }
      this.controlEnabled = value;
    });
    this.connectSliderElement(
      'simRateSlider',
      0.0,
      1.0,
      0.01,
      this.simRate,
      (value) => (this.simRate = value)
    );
    this.connectSliderElement(
      'baseMassSlider',
      0.1,
      2.0,
      0.01,
      this.dynamicsParams.m_b,
      (value) => (this.dynamicsParams.m_b = value),
      'kg'
    );
    this.connectSliderElement(
      'poleMassSlider',
      0.1,
      1.0,
      0.01,
      this.dynamicsParams.m_1,
      (value) => (this.dynamicsParams.m_1 = value),
      'kg'
    );
    this.connectSliderElement(
      'armLengthSlider',
      0.05,
      0.5,
      0.01,
      this.dynamicsParams.l_1,
      (value) => (this.dynamicsParams.l_1 = value),
      'm'
    );
    this.connectSliderElement(
      'cartFrictionSlider',
      0.01,
      0.5,
      0.01,
      this.dynamicsParams.mu_b,
      (value) => (this.dynamicsParams.mu_b = value)
    );
    this.connectSliderElement(
      'massDragSlider',
      0.01,
      0.15,
      0.01,
      this.dynamicsParams.c_d_1,
      (value) => (this.dynamicsParams.c_d_1 = value),
      'N/(m/s)<sup>2</sup>'
    );
    this.connectSliderElement(
      'cartPenaltySlider',
      0.0,
      200.0,
      1.0,
      this.optimizationParams.b_x_final_penalty,
      (value) => {
        this.optimizationParams.b_x_final_penalty = value;
        this.updatedOptimizationParams();
      },
      undefined,
      'change'
    );
    this.connectCheckbox('multipleShootingCheckbox', (value) => {
      if (value) {
        // Hybrid multiple-shooting with spacing of 10 inputs.
        this.optimizationParams.state_spacing = 10;
      } else {
        // Single-shooting.
        this.optimizationParams.state_spacing = this.optimizationParams.window_length;
      }
      this.updatedOptimizationParams();
    });

    const saveLogButton = document.getElementById('saveLogButton') as HTMLButtonElement;
    saveLogButton.addEventListener('click', () => {
      // Smash all the json bits together into a single log file.
      // TODO: This should be async probably.
      const logContent = `[${this.loggedMessages.join(', ')}]`;
      const blob = new Blob([logContent], { type: 'text/plain;charset=utf-8' });
      saveAs(blob, 'log.json');
    });

    // Show the "Save traces" button in development mode.
    if (this.wasm.isTracingEnabled()) {
      const saveTracesButton = document.getElementById('saveTracesButton') as HTMLButtonElement;
      saveTracesButton.parentElement.classList.remove('invisible');
      saveTracesButton.addEventListener('click', () => {
        const traces = this.wasm.getTraces();
        if (traces.length > 0) {
          const blob = new Blob([traces], { type: 'text/plain;charset=utf-8' });
          saveAs(blob, 'traces.json');
        }
      });
    }
  }

  private updatedOptimizationParams() {
    if (this.optimizer) {
      this.optimizer.delete();
    }
    this.optimizer = new this.wasm.Optimization(this.optimizationParams);
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
    this.accumulatedTime += durationInSeconds * this.simRate;
    while (this.accumulatedTime >= controlDt) {
      this.stepControlAndSim(controlDt);
      this.decayExternalForces(controlDt);
      this.accumulatedTime -= controlDt;
    }

    const currentState = this.simulator.getState() as SingleCartPoleState;
    const interaction = this.mouseHandler.determineInteraction(
      currentState,
      this.dynamicsParams,
      this.renderer.getPixelFromMetricTransform()
    );
    this.addExternalForce(interaction);

    this.renderer.drawSingle(currentState, this.dynamicsParams, interaction);
    this.controlPlotter.draw();

    this.previousTime = timestamp;
    this.requestFrame();
    this.iteration++;
  }

  // Run the MPC and simulator.
  private stepControlAndSim(dt: number) {
    // Run the model predictive controller.
    const outputs = this.optimizationTicToc.measureSpan(() => {
      return this.optimizer.step(this.simulator.getState(), this.dynamicsParams);
    });

    // Update the logged state.
    // We apply a limit on the history length to avoid exhausting memory.
    this.loggedMessages.push(outputs.toJson());
    if (this.loggedMessages.length > 5000) {
      this.loggedMessages.shift();
    }

    const enableTiming = false;
    if (enableTiming && this.iteration > 0 && this.iteration % 500 == 0) {
      const { max: max, mean: mean } = this.optimizationTicToc.computeStats();
      console.log(`Optimization times: mean = ${mean}, max = ${max}`);
    }

    // Step the sim forward. We only apply the control if the checkbox is checked.
    this.simulator.step(
      this.dynamicsParams,
      dt,
      this.controlEnabled ? outputs.getControl(0) : 0.0,
      this.externalForces
    );

    this.updatePlots(outputs);

    // We need to manually clean up C++ objects allocated via embind.
    outputs.delete();
  }

  // Apply exponential decay to the external forces (applied by user clicking).
  private decayExternalForces(dt: number) {
    const timeConstant = 0.1;
    const clip = (v: number) => {
      return Math.abs(v) < 1.0e-6 ? 0 : v;
    };
    this.externalForces = this.externalForces.map((p) => {
      return {
        x: clip(p.x * Math.max(0, 1 - dt / timeConstant)),
        y: clip(p.y * Math.max(0, 1 - dt / timeConstant))
      };
    });
  }

  private addExternalForce(interaction: MouseInteraction | null) {
    if (interaction == null || !interaction.clicked) {
      return;
    }

    // We apply 10x whatever the mass of the contact point is:
    const masses = [this.dynamicsParams.m_b, this.dynamicsParams.m_1];
    const normalizedForceMagnitude = 10.0 * masses[interaction.massIndex];

    // Angle is in canvas-space, so swap the y-axis into metric:
    console.assert(interaction.massIndex < this.externalForces.length);
    this.externalForces[interaction.massIndex] = {
      x: -Math.cos(interaction.incidentAngle) * normalizedForceMagnitude,
      y: Math.sin(interaction.incidentAngle) * normalizedForceMagnitude
    };
  }

  private updatePlots(outputs: OptimizationOutputs) {
    let times: Float64Array = new Float64Array(outputs.windowLength());
    for (let t = 0; t < outputs.windowLength(); ++t) {
      times[t] = t * 0.01;
    }

    let u_controls: Float64Array = new Float64Array(outputs.windowLength());
    // let x_states: Array<SingleCartPoleState> = [];
    times.forEach((_, index) => {
      u_controls[index] = outputs.getControl(index);
      // x_states.push(outputs.getPredictedState(index) as SingleCartPoleState);
    });

    // const angles = new Float64Array(
    //   x_states.map((x: SingleCartPoleState, _) => {
    //     return x.th_1;
    //   })
    // );

    this.controlPlotter.data = { x: times, y: u_controls };
  }
}

document.addEventListener('DOMContentLoaded', () => {
  OptimizationWasm().then((mod: MainModule) => {
    console.log('Loaded WASM module.');
    const app = new Application(mod);
    // Start the animation loop:
    app.requestFrame();
  });
});

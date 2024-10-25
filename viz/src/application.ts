// Copyright 2024 Gareth Cross.
import { saveAs } from 'file-saver';

import './styles.css';

import OptimizationWasm, {
  MainModule,
  Simulator,
  Optimization,
  OptimizationOutputs
} from './optimization-wasm';
import { Renderer } from './renderer';
import { Range, Plotter } from './plotter';
import { MouseHandler, MouseInteraction } from './input';
import { Point, SingleCartPoleState, SingleCartPoleParams, OptimizationParams } from './interfaces';
import { getUiHtml } from './ui';

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

  // The set-point for the cart location.
  private cartSetPoint: number = 0.0;

  // Predicted states from the MPC at the last timestep.
  private predictedStates: Array<SingleCartPoleState> = [];

  // Timing control:
  private previousTime: DOMHighResTimeStamp | null = null;
  private accumulatedTime: number = 0.0;
  private totalTime: number = 0.0;

  // Plotters
  private controlPlotter: Plotter;
  private anglePlotter: Plotter;
  private speedPlotter: Plotter;

  // An array of logged optimization outputs that we can dump to disk.
  private loggedMessages: Array<string> = [];

  // UI controls
  private controlEnabled: boolean = true;
  private simRate: number = 1.0;

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
      limitsY: new Range(-150.0, 150.0),
      gridX: {
        numMajorTicks: 5,
        numMinorTicks: 0
      },
      gridY: {
        numMajorTicks: 5,
        numMinorTicks: 0
      }
    });
    this.anglePlotter = new Plotter('anglePlot', {
      limitsY: new Range(-180.0, 180.0),
      gridX: {
        numMajorTicks: 5,
        numMinorTicks: 0
      },
      gridY: {
        numMajorTicks: 5,
        numMinorTicks: 0
      }
    });
    this.speedPlotter = new Plotter('speedPlot', {
      limitsY: new Range(-5.0, 5.0),
      gridX: {
        numMajorTicks: 5,
        numMinorTicks: 0
      },
      gridY: {
        numMajorTicks: 5,
        numMinorTicks: 0
      }
    });
    this.connectUi();
  }

  // Deallocate C++ resources.
  // This is for use when running with `-fsanitize=address`.
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
      'cartSetPointSlider',
      -1.0,
      1.0,
      0.01,
      this.cartSetPoint,
      (value) => (this.cartSetPoint = value),
      'm'
    );
    this.connectSliderElement(
      'cartPenaltySlider',
      0.0,
      200.0,
      1.0,
      this.optimizationParams.b_x_final_cost_weight,
      (value) => {
        this.optimizationParams.b_x_final_cost_weight = value;
        this.updatedOptimizationParams();
      },
      undefined,
      'change'
    );

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

  // When updating optimization params we need to tear down the C++ optimizer and reconstruct it.
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
      this.predictedStates = this.stepControlAndSim(controlDt);
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

    this.renderer.drawSingle(
      currentState,
      this.dynamicsParams,
      this.controlEnabled ? this.predictedStates : [],
      interaction
    );
    this.controlPlotter.draw();
    this.anglePlotter.draw();
    this.speedPlotter.draw();

    this.previousTime = timestamp;
    this.requestFrame();
  }

  // Run the MPC and simulator.
  private stepControlAndSim(dt: number): Array<SingleCartPoleState> {
    // Run the model predictive controller.
    const currentState = this.simulator.getState() as SingleCartPoleState;
    const outputs = this.optimizer.step(currentState, this.dynamicsParams, this.cartSetPoint);

    // Update the logged state.
    // We apply a limit on the history length to avoid exhausting memory.
    this.loggedMessages.push(outputs.toJson());
    if (this.loggedMessages.length > 5000) {
      this.loggedMessages.shift();
    }

    // Step the sim forward. We only apply the control if the checkbox is checked.
    this.simulator.step(
      this.dynamicsParams,
      dt,
      this.controlEnabled ? outputs.getControl(0) : 0.0,
      this.externalForces
    );

    const currentTime = this.totalTime;
    this.totalTime += dt;

    this.updatePlots(currentTime, currentState, outputs);

    // Get the predicted states out:
    var predictedStates: Array<SingleCartPoleState> = [];
    for (var i = 0; i < outputs.windowLength(); ++i) {
      predictedStates.push(outputs.getPredictedState(i));
    }

    // We need to manually clean up C++ objects allocated via embind.
    outputs.delete();
    return predictedStates;
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

  private appendToPlotter(plotter: Plotter, x: number, y: number, maxLength: number) {
    if (!plotter.data) {
      plotter.data = {
        x: new Float64Array([x]),
        y: new Float64Array([y])
      };
    } else {
      const sliceStart = plotter.data.x.length > maxLength ? 1 : 0;
      plotter.data.x = new Float64Array([...plotter.data.x.slice(sliceStart), x]);
      plotter.data.y = new Float64Array([...plotter.data.y.slice(sliceStart), y]);
    }
  }

  private updatePlots(
    currentTime: number,
    currentState: SingleCartPoleState,
    outputs: OptimizationOutputs
  ) {
    this.appendToPlotter(
      this.controlPlotter,
      currentTime,
      outputs.getControl(0),
      outputs.windowLength()
    );
    this.appendToPlotter(
      this.anglePlotter,
      currentTime,
      (currentState.th_1 * 180) / Math.PI,
      outputs.windowLength()
    );
    this.appendToPlotter(
      this.speedPlotter,
      currentTime,
      currentState.b_x_dot,
      outputs.windowLength()
    );
  }
}

// Main entry-point of the application.
// This should be called from: document.addEventListener('DOMContentLoaded', () => {...});
export function loadApplication(parentElementId: string) {
  // Insert HTML into the page:
  const parent = document.getElementById(parentElementId);
  if (parent) {
    parent.innerHTML = getUiHtml() as string;
  } else {
    console.error(`Could not find element: ${parent}`);
    return;
  }
  OptimizationWasm().then((mod: MainModule) => {
    const app = new Application(mod);
    // Start the animation loop:
    console.log('Starting WASM application.');
    app.requestFrame();
  });
}

import * as mathjs from 'mathjs';

import { PendulumState } from './simulator';
import { PendulumParams } from './pendulum_params';

// A canvas renderer for our pendulum simulator.
export class Renderer {
  private canvas: HTMLCanvasElement;
  private context: CanvasRenderingContext2D;

  constructor() {
    const canvas: HTMLCanvasElement = document.getElementById(
      'canvas'
    ) as HTMLCanvasElement;
    const context: CanvasRenderingContext2D = canvas.getContext('2d');

    this.canvas = canvas;
    this.context = context;
  }

  // Draw the current pendulum configuration.
  public draw(state: PendulumState, pendulum_params: PendulumParams) {
    const [ctxWidth, ctxHeight] = [this.canvas.width, this.canvas.height];
    this.context.clearRect(0, 0, ctxWidth, ctxHeight);

    // The width of the viewport, in meters:
    const viewportWidthMeters = 2.0 as const;
    const pixelsPerMeter = ctxWidth / viewportWidthMeters;

    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = 'black';
    this.context.lineWidth = 1;

    // Draw the horizontal axis :
    this.context.beginPath();
    this.context.moveTo(0.0, ctxHeight / 2.0);
    this.context.lineTo(ctxWidth, ctxHeight / 2.0);
    this.context.stroke();

    // Draw the pendulum base:
    const bx = pixelsPerMeter * state.b_x + ctxWidth / 2.0;
    const by = ctxHeight / 2.0;

    const baseWidthPixels = 0.1 * pixelsPerMeter;
    const baseHeightPixels = 0.1 * pixelsPerMeter;

    this.context.fillStyle = 'rgb(193 41 46)';
    this.context.beginPath();
    this.context.fillRect(
      bx - baseWidthPixels * 0.5,
      by - baseHeightPixels * 0.5,
      baseWidthPixels,
      baseHeightPixels
    );

    const x1 =
      bx + pixelsPerMeter * pendulum_params.l_1 * mathjs.cos(state.theta_1);
    const y1 =
      by - pixelsPerMeter * pendulum_params.l_1 * mathjs.sin(state.theta_1);

    const x2 =
      x1 + pixelsPerMeter * pendulum_params.l_2 * mathjs.cos(state.theta_2);
    const y2 =
      y1 - pixelsPerMeter * pendulum_params.l_2 * mathjs.sin(state.theta_2);

    // Draw the arms:
    this.context.lineWidth = 4;
    this.context.beginPath();
    this.context.moveTo(bx, by);
    this.context.lineTo(x1, y1);
    this.context.lineTo(x2, y2);
    this.context.stroke();

    // Draw the point masses:
    const pointRadiusPixels = 6.0;
    this.context.fillStyle = 'rgb(35 87 137)';
    this.context.beginPath();
    this.context.arc(x1, y1, pointRadiusPixels, 0, 2 * Math.PI, false);
    this.context.fill();
    this.context.beginPath();
    this.context.arc(x2, y2, pointRadiusPixels, 0, 2 * Math.PI, false);
    this.context.fill();
  }
}

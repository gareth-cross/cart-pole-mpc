import { SingleCartPoleParams } from './optimization-wasm';
import {
  massLocationsFromState,
  Point,
  ScaleAndTranslate,
  SingleCartPoleState
} from './interfaces';
import { MouseInteraction } from './input';

// A canvas renderer for our pendulum simulator.
export class Renderer {
  private canvas: HTMLCanvasElement;
  private context: CanvasRenderingContext2D;

  constructor() {
    const canvas: HTMLCanvasElement = document.getElementById('canvas') as HTMLCanvasElement;
    const context: CanvasRenderingContext2D = canvas.getContext('2d');

    this.canvas = canvas;
    this.context = context;

    // Automatically resize when the parent div changes size:
    const canvasParent = this.canvas.parentElement as HTMLDivElement;
    new ResizeObserver(() => this.parentSizeChanged()).observe(canvasParent);
    this.parentSizeChanged();
  }

  private parentSizeChanged() {
    this.canvas.width = this.canvas.offsetWidth;
    this.canvas.height = this.canvas.offsetHeight;
  }

  // Get the transformation from metric units to pixels in the canvas.
  public getPixelFromMetricTransform(): ScaleAndTranslate {
    // The width of the viewport, in meters:
    const [ctxWidth, ctxHeight] = [this.canvas.width, this.canvas.height];
    const viewportWidthMeters = 2.0 as const;
    const pixelsPerMeter = ctxWidth / viewportWidthMeters;

    return new ScaleAndTranslate(
      pixelsPerMeter,
      -pixelsPerMeter, //  Flip y-axis.
      ctxWidth / 2.0,
      ctxHeight / 2.0
    );
  }

  // Draw the current pendulum configuration.
  public drawSingle(
    state: SingleCartPoleState,
    dynamicsParams: SingleCartPoleParams,
    interaction: MouseInteraction | null
  ) {
    this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // Get transformation from metric to pixels.
    const pixelFromMetric = this.getPixelFromMetricTransform();

    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = 'black';
    this.context.lineWidth = 1;

    // Draw the horizontal axis :
    this.context.beginPath();
    this.context.moveTo(0.0, this.canvas.height / 2.0);
    this.context.lineTo(this.canvas.width, this.canvas.height / 2.0);
    this.context.stroke();

    // Draw the pendulum base:
    const [base, mass] = massLocationsFromState(state, dynamicsParams);

    const baseScaled = pixelFromMetric.transform(base);
    const massScaled = pixelFromMetric.transform(mass);

    const baseWidthPixels = 0.1 * pixelFromMetric.sx;
    const baseHeightPixels = 0.1 * pixelFromMetric.sy;

    this.context.fillStyle = 'rgb(193 41 46)';
    this.context.beginPath();
    this.context.fillRect(
      baseScaled.x - baseWidthPixels * 0.5,
      baseScaled.y - baseHeightPixels * 0.5,
      baseWidthPixels,
      baseHeightPixels
    );

    // Draw the arm:
    this.context.lineWidth = 4;
    this.context.beginPath();
    this.context.moveTo(baseScaled.x, baseScaled.y);
    this.context.lineTo(massScaled.x, massScaled.y);
    this.context.stroke();

    // Draw the point mass:
    const pointRadiusPixels = 6.0;
    this.context.fillStyle = 'rgb(35 87 137)';
    this.context.beginPath();
    this.context.arc(massScaled.x, massScaled.y, pointRadiusPixels, 0, 2 * Math.PI, false);
    this.context.fill();

    if (interaction != null) {
      const locations = [baseScaled, massScaled];
      this.drawMouseIndicator(locations[interaction.massIndex], interaction.incidentAngle);
    }
  }

  // Draw an arrow to indicate the mouse location and direction of force that will be applied.
  private drawMouseIndicator(massLocation: Point, angle: number) {
    const arrowLength = 50.0;
    const arrowDistanceFromMass = 20.0;
    const head = {
      x: massLocation.x + Math.cos(angle) * arrowDistanceFromMass,
      y: massLocation.y + Math.sin(angle) * arrowDistanceFromMass
    };
    const tail = {
      x: head.x + Math.cos(angle) * (arrowLength + arrowDistanceFromMass),
      y: head.y + Math.sin(angle) * (arrowLength + arrowDistanceFromMass)
    };
    this.drawArrow(head, tail);
  }

  private drawArrow(head: Point, tail: Point) {
    const length = Math.sqrt(Math.pow(head.x - tail.x, 2) + Math.pow(head.y - tail.y, 2));
    const headWidth = length * 0.25;
    const headLength = length * 0.25;
    const bodyWidth = length * 0.1;

    const tangent: Point = { x: (tail.x - head.x) / length, y: (tail.y - head.y) / length };
    const normal: Point = { x: -tangent.y, y: tangent.x };

    // Start drawing from the tip:
    this.context.save();
    this.context.strokeStyle = '#fff7ed';
    this.context.fillStyle = '#fb923c';
    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.shadowColor = 'rgba(0, 0, 0, 0.5)';
    this.context.shadowBlur = 10;
    this.context.shadowOffsetX = 5;
    this.context.shadowOffsetY = 5;

    this.context.beginPath();
    this.context.moveTo(head.x, head.y);
    this.context.lineTo(
      head.x + tangent.x * headLength + normal.x * headWidth,
      head.y + tangent.y * headLength + normal.y * headWidth
    );
    this.context.lineTo(
      head.x + tangent.x * headLength + normal.x * bodyWidth,
      head.y + tangent.y * headLength + normal.y * bodyWidth
    );

    this.context.lineTo(
      head.x + tangent.x * length + normal.x * bodyWidth,
      head.y + tangent.y * length + normal.y * bodyWidth
    );
    this.context.lineTo(
      head.x + tangent.x * length - normal.x * bodyWidth,
      head.y + tangent.y * length - normal.y * bodyWidth
    );

    this.context.lineTo(
      head.x + tangent.x * headLength - normal.x * bodyWidth,
      head.y + tangent.y * headLength - normal.y * bodyWidth
    );
    this.context.lineTo(
      head.x + tangent.x * headLength - normal.x * headWidth,
      head.y + tangent.y * headLength - normal.y * headWidth
    );
    this.context.closePath();
    this.context.fill();
    this.context.stroke();
    this.context.restore();
  }
}

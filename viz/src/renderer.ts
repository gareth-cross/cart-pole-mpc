// Copyright (c) 2024 Gareth Cross.
import { Point, SingleCartPoleState, SingleCartPoleParams } from './interfaces';
import { MouseInteraction } from './input';
import { massLocationsFromState, ScaleAndTranslate } from './utils';

// A canvas renderer for our pendulum simulator.
export class Renderer {
  private canvas: HTMLCanvasElement;
  private context: CanvasRenderingContext2D;
  private viewportWidthMeters: number;

  constructor() {
    const canvas: HTMLCanvasElement = document.getElementById(
      'cartPoleCanvas'
    ) as HTMLCanvasElement;
    const context: CanvasRenderingContext2D = canvas.getContext('2d');

    this.canvas = canvas;
    this.context = context;
    this.viewportWidthMeters = 2.0;

    // Do not try to select text when double clicking the canvas:
    this.canvas.onselectstart = function () {
      return false;
    };

    // Stop the copy/share dialog from popping up on mobile.
    this.canvas.addEventListener('contextmenu', (e) => {
      e.returnValue = false;
      e.preventDefault();
    });

    // Automatically resize when the parent div changes size:
    const canvasParent = this.canvas.parentElement as HTMLDivElement;
    canvasParent.onselectstart = function () {
      return false;
    };
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
    const pixelsPerMeter = ctxWidth / this.viewportWidthMeters;
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
    predictedStates: Array<SingleCartPoleState>,
    interaction: MouseInteraction | null
  ) {
    this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // Get transformation from metric to pixels.
    const pixelFromMetric = this.getPixelFromMetricTransform();

    const cartHeightMeters = 0.2;
    const floorYMeters = -cartHeightMeters * 0.5;
    const floorY = pixelFromMetric.transform({ x: 0, y: floorYMeters }).y;

    // Draw the horizontal axis:
    this.context.save();
    this.context.lineCap = 'butt';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = '#FBA108';
    this.context.lineWidth = 2;
    this.context.beginPath();
    this.context.moveTo(0.0, floorY);
    this.context.lineTo(this.canvas.width, floorY);
    this.context.stroke();
    this.context.restore();

    // Draw the predicted states:
    const drawGhosts = true;
    if (drawGhosts) {
      const predictedStateStep = 10;
      for (var i = 0; i < predictedStates.length; i += predictedStateStep) {
        this.drawCart(
          predictedStates[i],
          dynamicsParams,
          pixelFromMetric,
          cartHeightMeters,
          1.0 - (i + 0.5) / predictedStates.length
        );
      }
    }

    // Draw the cart at the current state:
    this.drawCart(state, dynamicsParams, pixelFromMetric, cartHeightMeters, 1.0);

    // Some lines to give the floor a bit of contrast:
    this.context.save();
    const numFloorLines = 20;
    for (var i = 0; i < numFloorLines; ++i) {
      const x =
        ((i + 0.5) / numFloorLines) * this.viewportWidthMeters - this.viewportWidthMeters * 0.5;
      const originPt = pixelFromMetric.transform({ x: x, y: floorYMeters });
      const terminalPt = pixelFromMetric.transform({ x: x - 0.1, y: floorYMeters - 0.1 });

      this.context.lineCap = 'round';
      var grad = this.context.createLinearGradient(
        originPt.x,
        originPt.y + 1.0,
        terminalPt.x,
        terminalPt.y + 1.0
      );
      grad.addColorStop(0, 'rgba(251, 161, 8, 1.0)');
      grad.addColorStop(1, 'rgba(251, 161, 8, 0.25)');
      this.context.strokeStyle = grad;
      this.context.setLineDash([5, 5]);
      this.context.beginPath();
      this.context.moveTo(originPt.x, originPt.y + 1.0);
      this.context.lineTo(terminalPt.x, terminalPt.y + 1.0);
      this.context.stroke();
      this.context.setLineDash([]);
    }
    this.context.restore();

    if (interaction != null) {
      const [base, mass] = massLocationsFromState(state, dynamicsParams);
      const baseScaled = pixelFromMetric.transform(base);
      const massScaled = pixelFromMetric.transform(mass);
      const locations = [baseScaled, massScaled];
      this.drawMouseIndicator(
        locations[interaction.massIndex],
        interaction.incidentAngle,
        interaction.clicked
      );
    }
  }

  // Draw the cart at the provided `state`.
  private drawCart(
    state: SingleCartPoleState,
    dynamicsParams: SingleCartPoleParams,
    pixelFromMetric: ScaleAndTranslate,
    cartHeightMeters: number,
    alpha: number
  ) {
    // Draw the pendulum base:
    const [base, mass] = massLocationsFromState(state, dynamicsParams);

    // Draw the cart body:
    const bodyPts = [
      { x: 0, y: 1 },
      { x: 1, y: 1 },
      { x: 2, y: 3 },
      { x: 4, y: 3 },
      { x: 5, y: 1 },
      { x: 7, y: 1 },
      { x: 8, y: 3 },
      { x: 10, y: 3 },
      { x: 11, y: 1 },
      { x: 12, y: 1 },
      { x: 12, y: 6 },
      { x: 11, y: 7 },
      { x: 1, y: 7 },
      { x: 0, y: 6 }
    ];

    // Draw outline of the body (in clockwise order).
    const drawBodyTrace = () => {
      bodyPts.forEach((p: Point, index: number) => {
        const q = pixelFromMetric.transform(worldFromCart.transform(p));
        if (index == 0) {
          this.context.moveTo(q.x, q.y);
        } else {
          this.context.lineTo(q.x, q.y);
        }
      });
    };

    const wheelPositions = [
      { x: 3, y: 2 },
      { x: 9, y: 2 }
    ];

    const cartScale = cartHeightMeters / 7;
    const worldFromCart = new ScaleAndTranslate(
      cartScale,
      cartScale,
      -cartScale * 6 + base.x,
      -cartScale * 3.5 + base.y
    );

    // Draw wheels first
    this.context.save();

    if (alpha < 1.0) {
      this.context.save();
      this.context.beginPath();
      this.context.rect(0, 0, this.canvas.width, this.canvas.height);
      drawBodyTrace();
      this.context.clip();
    }

    wheelPositions.forEach((p) => {
      const q = pixelFromMetric.transform(worldFromCart.transform(p));
      this.context.beginPath();
      this.context.arc(q.x, q.y, pixelFromMetric.sx * 0.05, 0, 2 * Math.PI, false);
      this.context.fillStyle = `rgba(82, 82, 82, ${alpha})`;
      this.context.fill();
      this.context.strokeStyle = `rgba(229, 229, 229, ${alpha})`;
      this.context.lineWidth = 2;
      this.context.stroke();
    });

    if (alpha < 1.0) {
      this.context.restore();
    }

    // Then body
    this.context.fillStyle = `rgba(15, 118, 110, ${alpha})`;
    this.context.shadowColor = `rgba(0, 0, 0, ${alpha < 1.0 ? 0.0 : 0.5})`;
    this.context.shadowBlur = 5;
    this.context.shadowOffsetX = 0;
    this.context.shadowOffsetY = 0;
    this.context.beginPath();
    drawBodyTrace();
    this.context.fill();
    this.context.restore();

    const baseScaled = pixelFromMetric.transform(base);
    const massScaled = pixelFromMetric.transform(mass);

    const pointRadiusPixels = pixelFromMetric.sx * 0.03;

    // Clipping mask on the mass to clip the arm when alpha blending.
    // A bit lazy, we could just truncate the arm.
    if (alpha < 1.0) {
      this.context.save();
      this.context.beginPath();
      this.context.rect(0, 0, this.canvas.width, this.canvas.height);
      this.context.arc(massScaled.x, massScaled.y, pointRadiusPixels, 0, 2 * Math.PI, true);
      this.context.clip();
    }

    // Draw the arm:
    this.context.strokeStyle = `rgba(229, 229, 229, ${alpha})`;
    this.context.lineWidth = 3;
    this.context.lineCap = 'round';
    this.context.beginPath();
    this.context.moveTo(baseScaled.x, baseScaled.y);
    this.context.lineTo(massScaled.x, massScaled.y);
    this.context.stroke();
    if (alpha < 1.0) {
      this.context.restore(); // Pop the clipping mask.
    }

    // Draw the point mass:
    this.context.fillStyle = `rgba(195, 57, 27, ${alpha})`;
    this.context.strokeStyle = `rgba(229, 229, 229, ${alpha})`;
    this.context.beginPath();
    this.context.arc(massScaled.x, massScaled.y, pointRadiusPixels, 0, 2 * Math.PI, false);
    this.context.fill();
    this.context.lineWidth = 2;
    this.context.stroke();

    this.context.restore();
  }

  // Draw an arrow to indicate the mouse location and direction of force that will be applied.
  private drawMouseIndicator(massLocation: Point, angle: number, clicked: boolean) {
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
    this.drawArrow(head, tail, clicked);
  }

  private drawArrow(head: Point, tail: Point, depressed: boolean) {
    const length = Math.sqrt(Math.pow(head.x - tail.x, 2) + Math.pow(head.y - tail.y, 2));
    const headWidth = length * 0.25;
    const headLength = length * 0.25;
    const bodyWidth = length * 0.1;

    const tangent: Point = { x: (tail.x - head.x) / length, y: (tail.y - head.y) / length };
    const normal: Point = { x: -tangent.y, y: tangent.x };

    // Start drawing from the tip:
    this.context.save();
    this.context.strokeStyle = '#fff7ed';
    this.context.fillStyle = depressed ? '#c2410c' : '#fb923c';
    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.lineWidth = 4;

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

    if (!depressed) {
      this.context.shadowColor = 'rgba(0, 0, 0, 0.5)';
      this.context.shadowBlur = 10;
      this.context.shadowOffsetX = 5;
      this.context.shadowOffsetY = 5;
    } else {
      this.context.shadowColor = 'rgba(0, 0, 0, 0.0)';
    }
    this.context.fill();

    this.context.shadowColor = 'rgba(0, 0, 0, 0.0)';
    this.context.stroke();

    this.context.restore();
  }
}

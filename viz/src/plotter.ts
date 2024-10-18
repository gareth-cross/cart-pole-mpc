// Copyright 2024 Gareth Cross.
import { ScaleAndTranslate, Point } from './interfaces';

class Range {
  public min: number;
  public max: number;

  constructor(min: number, max: number) {
    this.min = min;
    this.max = max;
  }

  public convertFrom(v: number): number {
    return (v - this.min) / (this.max - this.min);
  }

  public convertTo(v: number): number {
    return v * (this.max - this.min) + this.min;
  }
}

export interface GridTickConfig {
  majorInterval: number;
  numMinorTicks: number;
}

// Parameters we can pass to `Plotter` to configure rendering.
export interface PlotterConfig {
  yAxisLimitLower?: number;
  yAxisLimitUpper?: number;

  // Spacing between grid-ticks on the x and y axes.
  gridX?: GridTickConfig;
  gridY?: GridTickConfig;
}

export interface TickMark {
  position: number;
  major: boolean;
}

// A simple plotter.
export class Plotter {
  private canvas: HTMLCanvasElement;
  private overlayDiv: HTMLDivElement;
  private context: CanvasRenderingContext2D;
  private config: PlotterConfig;
  private mousePosition: Point | null = null;

  // Plot data we can render in this context.
  public data: { x: Float64Array; y: Float64Array } | null = null;

  constructor(parent_id: string, config: PlotterConfig) {
    const parent: HTMLElement = document.getElementById(parent_id) as HTMLElement;
    console.assert(parent != null, `Element with id ${parent_id} does not exist.`);

    this.canvas = document.createElement('canvas') as HTMLCanvasElement;
    this.canvas.style.position = 'absolute';
    this.canvas.style.width = '100%';
    this.canvas.style.height = '320px';
    this.context = this.canvas.getContext('2d');
    parent.appendChild(this.canvas);

    this.overlayDiv = document.createElement('div') as HTMLDivElement;
    this.overlayDiv.style.position = 'absolute';
    this.overlayDiv.style.right = '10px';
    parent.appendChild(this.overlayDiv);

    this.config = config;

    // Automatically resize when the parent div changes size:
    new ResizeObserver(() => this.parentSizeChanged()).observe(parent);
    this.parentSizeChanged();

    this.canvas.addEventListener('mouseleave', (e) => {
      this.mouseLeave(e);
    });
    this.canvas.addEventListener('mousemove', (e) => {
      this.mouseMove(e);
    });
  }

  private parentSizeChanged() {
    this.canvas.width = this.canvas.offsetWidth;
    this.canvas.height = this.canvas.offsetHeight;
  }

  private computeAxisBounds(): { x: Range; y: Range } {
    console.assert(this.data != null);
    const { x: x, y: y } = this.data;

    const yMin = this.config.yAxisLimitLower || Number.POSITIVE_INFINITY;
    const yMax = this.config.yAxisLimitUpper || Number.NEGATIVE_INFINITY;

    return {
      x: new Range(Math.min(...x), Math.max(...x)),
      y: new Range(yMin, yMax) //new Range(Math.min(Math.min(...y), y_min_bound), Math.max(Math.max(...y), y_max_bound))
    };
  }

  // Get the transform from normalized (bottom-left) coordinates to pixels (top-left).
  private getPixelFromNormalizedTransform(): ScaleAndTranslate {
    const [ctxWidth, ctxHeight] = [this.canvas.width, this.canvas.height];
    return new ScaleAndTranslate(ctxWidth, -ctxHeight, 0.0, ctxHeight);
  }

  // Draw the current pendulum configuration.
  public draw() {
    const [ctxWidth, ctxHeight] = [this.canvas.width, this.canvas.height];
    this.context.clearRect(0, 0, ctxWidth, ctxHeight);
    this.context.fillStyle = '#E6ECF6';
    this.context.fillRect(0, 0, ctxWidth, ctxHeight);

    if (this.data == null) {
      return;
    }

    const { x: rangeX, y: rangeY } = this.computeAxisBounds();
    const ticksX = this.computeGridMarkings(this.config.gridX, rangeX);
    const ticksY = this.computeGridMarkings(this.config.gridY, rangeY);

    // Draw the background grid.
    this.drawGrid(ticksX, ticksY, rangeX, rangeY);

    // Draw axes w/ tick marks.
    this.drawAxes(ticksX, ticksY, rangeX, rangeY);

    // Draw some data:
    const { x: xData, y: yData } = this.data;
    console.assert(
      xData.length == yData.length,
      `X & Y data have mismatched lengths: ${xData.length} != ${yData.length}`
    );

    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = '#C3391B';
    this.context.lineWidth = 2;
    this.context.setLineDash([]);

    const pixelsFromNorm = this.getPixelFromNormalizedTransform();
    const xDataScaled: Float64Array = xData.map((x) => {
      return pixelsFromNorm.transformX(rangeX.convertFrom(x));
    });
    const yDataScaled: Float64Array = yData.map((y) => {
      return pixelsFromNorm.transformY(rangeY.convertFrom(y));
    });

    this.context.beginPath();
    this.context.moveTo(xDataScaled[0], yDataScaled[0]);
    for (var i = 1; i < xDataScaled.length; ++i) {
      this.context.lineTo(xDataScaled[i], yDataScaled[i]);
    }
    this.context.stroke();

    // Draw hover indicator where the user selects.
    this.drawMouseReticule(rangeX, rangeY);
  }

  // Compute the positions where grid-markings should be drawn on the chart.
  // Returns values in normalized [0, 1] coordinates.
  private computeGridMarkings(config: GridTickConfig | null, range: Range): Array<TickMark> {
    var result: Array<TickMark> = new Array();
    if (config == null) {
      return result;
    }

    const { majorInterval: major, numMinorTicks: numMinorTicks } = config;
    console.assert(numMinorTicks >= 0 && major > 0, `Invalid grid params: ${this.config.gridX}`);

    for (var majorVal = range.min; majorVal < range.max; majorVal += major) {
      if (majorVal != range.min) {
        // Bit of a kludge, but don't push back the first major tick mark. The axis renderer will
        // draw on top of it anyways.
        result.push({ position: majorVal, major: true });
      }

      if (numMinorTicks > 0) {
        // Compute how much space between each tick.
        const spacing = major / (numMinorTicks + 1);
        for (var i = 0; i < numMinorTicks; ++i) {
          result.push({ position: (i + 1) * spacing + majorVal, major: false });
        }
      }
    }
    return result;
  }

  // Draw a grid on the plot area using the parameters specified in `x_grid` and `y_grid`.
  private drawGrid(ticksX: Array<TickMark>, ticksY: Array<TickMark>, rangeX: Range, rangeY: Range) {
    this.context.save();
    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = '#737373';

    const pixelsFromNorm = this.getPixelFromNormalizedTransform();

    ticksX.forEach((mark) => {
      const { position: x, major: isMajor } = mark;
      if (isMajor) {
        this.context.lineWidth = 0.5;
        this.context.setLineDash([]);
      } else {
        this.context.lineWidth = 0.25;
        this.context.setLineDash([5, 5]);
      }
      const start = pixelsFromNorm.transform({ x: rangeX.convertFrom(x), y: 0 });
      const end = pixelsFromNorm.transform({ x: rangeX.convertFrom(x), y: 1 });
      this.context.beginPath();
      this.context.moveTo(start.x, start.y);
      this.context.lineTo(end.x, end.y);
      this.context.stroke();
    });

    ticksY.forEach((mark) => {
      const { position: y, major: isMajor } = mark;
      if (isMajor) {
        this.context.lineWidth = 0.5;
        this.context.setLineDash([]);
      } else {
        this.context.lineWidth = 0.25;
        this.context.setLineDash([5, 5]);
      }
      const start = pixelsFromNorm.transform({ x: 0, y: rangeY.convertFrom(y) });
      const end = pixelsFromNorm.transform({ x: 1, y: rangeY.convertFrom(y) });
      this.context.beginPath();
      this.context.moveTo(start.x, start.y);
      this.context.lineTo(end.x, end.y);
      this.context.stroke();
    });
    this.context.restore();
  }

  private drawAxes(ticksX: Array<TickMark>, ticksY: Array<TickMark>, rangeX: Range, rangeY: Range) {
    const pixelsFromNorm = this.getPixelFromNormalizedTransform();

    const { x: x0, y: y0 } = pixelsFromNorm.transform({
      x: rangeX.convertFrom(0),
      y: rangeY.convertFrom(0)
    });

    // First we draw lines for the axes:
    this.context.strokeStyle = '#171717';
    this.context.fillStyle = '#171717';
    this.context.lineWidth = 2;
    this.context.lineCap = 'round';
    this.context.beginPath();
    this.context.moveTo(Math.max(x0, 1), 0);
    this.context.lineTo(Math.max(x0, 1), this.canvas.height);
    this.context.stroke();

    this.context.beginPath();
    this.context.moveTo(0, Math.max(y0, 1));
    this.context.lineTo(this.canvas.width, Math.max(y0, 1));
    this.context.stroke();

    // Now draw some tick marks on the major ticks:
    const tickLength = 10;
    ticksX.forEach((tick) => {
      if (tick.major) {
        const x = pixelsFromNorm.transformX(rangeX.convertFrom(tick.position));
        this.context.beginPath();
        this.context.moveTo(x, y0 + tickLength);
        this.context.lineTo(x, y0 - tickLength);
        this.context.stroke();
      }
    });

    ticksY.forEach((tick) => {
      if (tick.major) {
        const y = pixelsFromNorm.transformY(rangeY.convertFrom(tick.position));

        this.context.beginPath();
        this.context.moveTo(x0 + tickLength, y);
        this.context.lineTo(x0 - tickLength, y);
        this.context.stroke();
        // Looks bad:
        // this.context.font = '10px Arial';
        // this.context.fillText(`${tick.position}`, x0 + tickLength + 2, y - 10);
      }
    });
  }

  private drawMouseReticule(rangeX: Range, rangeY: Range) {
    if (this.mousePosition == null) {
      this.overlayDiv.innerHTML = '';
      return;
    }
    const { x: dataX, y: dataY } = this.data;

    const pixelsFromNormalized = this.getPixelFromNormalizedTransform();

    // Find the data sample closest to the mouse:
    var k = 0;
    var dMin = Number.POSITIVE_INFINITY;
    for (var i = 0; i < dataX.length; ++i) {
      const p = pixelsFromNormalized.transform({
        x: rangeX.convertFrom(dataX[i]),
        y: rangeY.convertFrom(dataY[i])
      });
      const d = Math.sqrt(
        Math.pow(p.x - this.mousePosition.x, 2) + Math.pow(p.y - this.mousePosition.y, 2)
      );
      if (d < dMin) {
        dMin = d;
        k = i;
      }
    }

    const pt: Point = { x: dataX[k], y: dataY[k] };
    const ptPixels = pixelsFromNormalized.transform({
      x: rangeX.convertFrom(pt.x),
      y: rangeY.convertFrom(pt.y)
    });

    this.context.save();
    this.context.strokeStyle = '#172554';

    // vertical line:
    this.context.lineWidth = 1.5;
    this.context.setLineDash([5, 5]);
    this.context.beginPath();
    this.context.moveTo(ptPixels.x, this.canvas.height);
    this.context.lineTo(ptPixels.x, ptPixels.y);
    this.context.stroke();

    this.context.beginPath();
    this.context.moveTo(0, ptPixels.y);
    this.context.lineTo(ptPixels.x, ptPixels.y);
    this.context.stroke();
    this.context.restore();

    // update the overlay
    this.overlayDiv.innerHTML = `x = ${pt.x}, y = ${pt.y.toPrecision(4)}`;
  }

  private mouseLeave(event: MouseEvent) {
    this.mousePosition = null;
  }

  private mouseMove(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
  }
}

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

// Parameters we can pass to `Plotter` to configure rendering.
export interface PlotterConfig {
  yaxis_limit_lower?: number;
  yaxis_limit_upper?: number;
}

// A simple plotter.
export class Plotter {
  private canvas: HTMLCanvasElement;
  private context: CanvasRenderingContext2D;
  private config: PlotterConfig;

  // Plot data we can render in this context.
  public data: { x: Float64Array; y: Float64Array } | null = null;

  constructor(parent_id: string, config: PlotterConfig) {
    const parent: HTMLElement = document.getElementById(parent_id) as HTMLElement;
    console.assert(parent != null, `Element with id ${parent_id} does not exist.`);

    this.canvas = document.createElement('canvas') as HTMLCanvasElement;
    this.canvas.style.width = '100%';
    this.canvas.style.height = '320px';
    this.context = this.canvas.getContext('2d');
    parent.appendChild(this.canvas);

    this.config = config;

    // Automatically resize when the parent div changes size:
    new ResizeObserver(() => this.parentSizeChanged()).observe(parent);
    this.parentSizeChanged();
  }

  private parentSizeChanged() {
    this.canvas.width = this.canvas.offsetWidth;
    this.canvas.height = this.canvas.offsetHeight;
  }

  private computeAxisBounds(): { x: Range; y: Range } {
    console.assert(this.data != null);
    const { x: x, y: y } = this.data;

    const y_min_bound = this.config.yaxis_limit_lower || Number.POSITIVE_INFINITY;
    const y_max_bound = this.config.yaxis_limit_upper || Number.NEGATIVE_INFINITY;

    return {
      x: new Range(Math.min(...x), Math.max(...x)),
      y: new Range(Math.min(Math.min(...y), y_min_bound), Math.max(Math.max(...y), y_max_bound))
    };
  }

  // Draw the current pendulum configuration.
  public draw() {
    const [ctxWidth, ctxHeight] = [this.canvas.width, this.canvas.height];
    this.context.clearRect(0, 0, ctxWidth, ctxHeight);

    if (this.data == null) {
      return;
    }

    const { x: x_range, y: y_range } = this.computeAxisBounds();

    const x_dst_range = new Range(0, ctxWidth);
    const y_dst_range = new Range(0, ctxHeight);

    // Draw some data:
    const { x: x_data, y: y_data } = this.data;
    console.assert(
      x_data.length == y_data.length,
      `X & Y data have mismatched lengths: ${x_data.length} != ${y_data.length}`
    );

    // console.log(y_data);

    this.context.lineCap = 'round';
    this.context.lineJoin = 'round';
    this.context.strokeStyle = '#C3391B';
    this.context.lineWidth = 2;

    const x_data_scaled: Float64Array = x_data.map((x) => {
      return x_dst_range.convertTo(x_range.convertFrom(x));
    });
    const y_data_scaled: Float64Array = y_data.map((y) => {
      return y_dst_range.convertTo(y_range.convertFrom(y));
    });

    this.context.beginPath();
    this.context.moveTo(x_data_scaled[0], y_data_scaled[0]);
    for (var i = 1; i < x_data_scaled.length; ++i) {
      this.context.lineTo(x_data_scaled[i], y_data_scaled[i]);
    }
    this.context.stroke();
  }
}

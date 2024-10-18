// Copyright 2024 Gareth Cross.

export interface SingleCartPoleState {
  b_x: number;
  th_1: number;
  b_x_dot: number;
  th_1_dot: number;
}

export interface SingleCartPoleParams {
  m_b: number;
  m_1: number;
  l_1: number;
  g: number;
  mu_b: number;
  v_mu_b: number;
  c_d_1: number;
}

export interface Point {
  x: number;
  y: number;
}

// A translation and scaling operation.
export class ScaleAndTranslate {
  public sx: number;
  public sy: number;
  public tx: number;
  public ty: number;

  constructor(sx: number, sy: number, tx: number, ty: number) {
    this.sx = sx;
    this.sy = sy;
    this.tx = tx;
    this.ty = ty;
  }

  public inverse(): ScaleAndTranslate {
    return new ScaleAndTranslate(1 / this.sx, 1 / this.sy, -this.tx / this.sx, -this.ty / this.sy);
  }

  public transformX(x: number): number {
    return this.sx * x + this.tx;
  }

  public transformY(y: number): number {
    return this.sy * y + this.ty;
  }

  public transform(p: Point): Point {
    return { x: this.transformX(p.x), y: this.transformY(p.y) };
  }
}

export function massLocationsFromState(
  state: SingleCartPoleState,
  params: SingleCartPoleParams
): [Point, Point] {
  const bx = state.b_x;
  const by = 0.0;
  const x1 = bx + params.l_1 * Math.cos(state.th_1);
  const y1 = by + params.l_1 * Math.sin(state.th_1);
  return [
    { x: bx, y: by },
    { x: x1, y: y1 }
  ];
}

export function indexOfSmallest<T>(a: Array<T>) {
  console.assert(a.length > 0);
  var lowest = 0;
  for (var i = 1; i < a.length; i++) {
    if (a[i] < a[lowest]) lowest = i;
  }
  return lowest;
}
